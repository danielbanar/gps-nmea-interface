#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_GPS_PORT "/dev/ttyS0"
#define DEFAULT_BAUDRATE B9600
#define GPS_DIR "/tmp/gps"

// Function to convert baudrate string to speed_t
speed_t get_baudrate(int baud)
{
    switch (baud)
    {
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    default:
        return B9600;
    }
}

// Function to configure the serial port
int configure_serial_port(int fd, speed_t baudrate)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0)
    {
        perror("Error getting serial port attributes");
        return -1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("Error setting serial port attributes");
        return -1;
    }

    return 0;
}

// Function to read a line from the serial port
int read_line(int fd, char* buffer, size_t buffer_size)
{
    int index = 0;
    char c;

    while (index < buffer_size - 1)
    {
        ssize_t bytes_read = read(fd, &c, 1);
        if (bytes_read == -1)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                usleep(10000); // Sleep for 10ms to avoid busy waiting
                continue;
            }
            perror("Error reading from serial port");
            return -1;
        }
        if (bytes_read == 0)
        {
            usleep(10000); // Sleep for 10ms if no data available
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            buffer[index] = '\0';
            return 0;
        }

        buffer[index++] = c;
    }

    buffer[index] = '\0';
    return 0;
}

int split_string(char* str, char* components[], int maxComponents)
{
    char* token;
    int fieldCount = 0;
    char* ptr = str;

    while (*ptr != '\0' && fieldCount < maxComponents)
    {
        token = ptr;
        while (*ptr != ',' && *ptr != '\0')
        {
            ptr++;
        }
        if (*ptr == ',')
        {
            *ptr = '\0';
            ptr++;
        }
        components[fieldCount++] = token;
    }

    return fieldCount;
}

// Function to convert coordinates from NMEA format to decimal degrees
float convert_to_decimal_degrees(float coordinate, char direction)
{
    int degrees = (int)(coordinate / 100);
    float minutes = coordinate - (degrees * 100);
    float decimal_degrees = degrees + (minutes / 60.0f);

    if (direction == 'S' || direction == 'W')
    {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

struct GPSData
{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int millisecond;
    double longitude;
    double latitude;
    double altitude;
    double speed_2d; // 2D speed (horizontal)
    double speed_3d; // 3D speed (horizontal + vertical)
    double knots;
    int satellites;
    time_t timestamp;
    int valid_time;            // Flag to indicate if time data is valid
    int valid_fix;             // Flag to indicate if GPS fix is valid
    double vertical_speed;     // Vertical speed in m/s
    double last_altitude;      // Previous altitude for vertical speed calculation
    time_t last_altitude_time; // Time of last altitude measurement
    double pdop;               // Position Dilution of Precision
    double hdop;               // Horizontal Dilution of Precision
    double vdop;               // Vertical Dilution of Precision
    int has_time;              // Flag to indicate if time data is available
};

struct GPSData gpsInfo;

// Function to write a value to a file in /tmp/gps
void write_to_file(const char* filename, const char* value)
{
    static char path[256];
    snprintf(path, sizeof(path), "%s/%s", GPS_DIR, filename);

    FILE* file = fopen(path, "w");
    if (file)
    {
        fprintf(file, "%s", value);
        fclose(file);
    }
}

// Function to calculate vertical speed
void calculate_vertical_speed()
{
    static time_t last_time = 0;
    static double last_altitude = 0;
    time_t current_time = time(NULL);

    if (last_time > 0 && current_time > last_time)
    {
        double time_diff = difftime(current_time, last_time);
        double altitude_diff = gpsInfo.altitude - last_altitude;
        gpsInfo.vertical_speed = altitude_diff / time_diff;
    }

    last_time = current_time;
    last_altitude = gpsInfo.altitude;
}

// Function to calculate 3D speed
void calculate_3d_speed()
{
    // 3D speed = sqrt(2D_speed^2 + vertical_speed^2)
    gpsInfo.speed_3d = sqrt(gpsInfo.speed_2d * gpsInfo.speed_2d +
                            gpsInfo.vertical_speed * gpsInfo.vertical_speed);
}

// Function to update GPS value files
void update_gps_files()
{
    static char value_str[64];
    static char datetime_str[64];
    static struct tm timeinfo;

    // Check if time data is valid (has time and reasonable year)
    int is_time_valid = gpsInfo.has_time && gpsInfo.year >= 2000 && gpsInfo.year <= 2100;

    // Check if GPS fix is valid (has coordinates, reasonable values, etc.)
    int is_fix_valid = is_time_valid &&
                       (gpsInfo.latitude >= -90 && gpsInfo.latitude <= 90) &&
                       (gpsInfo.longitude >= -180 && gpsInfo.longitude <= 180) &&
                       (gpsInfo.satellites >= 3) &&
                       (gpsInfo.hdop < 10.0 && gpsInfo.vdop < 10.0 && gpsInfo.pdop < 10.0);

    // Update validity flags
    gpsInfo.valid_time = is_time_valid;
    gpsInfo.valid_fix = is_fix_valid;

    // Write validity flags
    write_to_file("valid_time", is_time_valid ? "1" : "0");
    write_to_file("valid_fix", is_fix_valid ? "1" : "0");

    // Always write time data if available
    if (is_time_valid)
    {
        // Write date and time values
        snprintf(value_str, sizeof(value_str), "%04d", gpsInfo.year);
        write_to_file("year", value_str);

        snprintf(value_str, sizeof(value_str), "%02d", gpsInfo.month);
        write_to_file("month", value_str);

        snprintf(value_str, sizeof(value_str), "%02d", gpsInfo.day);
        write_to_file("day", value_str);

        snprintf(value_str, sizeof(value_str), "%02d", gpsInfo.hour);
        write_to_file("hour", value_str);

        snprintf(value_str, sizeof(value_str), "%02d", gpsInfo.minute);
        write_to_file("minute", value_str);

        snprintf(value_str, sizeof(value_str), "%02d", gpsInfo.second);
        write_to_file("second", value_str);

        snprintf(value_str, sizeof(value_str), "%03d", gpsInfo.millisecond);
        write_to_file("millisecond", value_str);

        // Write formatted date and time (without milliseconds)
        snprintf(datetime_str, sizeof(datetime_str), "%04d-%02d-%02d %02d:%02d:%02d",
                 gpsInfo.year, gpsInfo.month, gpsInfo.day,
                 gpsInfo.hour, gpsInfo.minute, gpsInfo.second);
        write_to_file("datetime", datetime_str);

        // Calculate and write timestamp
        timeinfo.tm_year = gpsInfo.year - 1900;
        timeinfo.tm_mon = gpsInfo.month - 1;
        timeinfo.tm_mday = gpsInfo.day;
        timeinfo.tm_hour = gpsInfo.hour;
        timeinfo.tm_min = gpsInfo.minute;
        timeinfo.tm_sec = gpsInfo.second;
        timeinfo.tm_isdst = -1;

        gpsInfo.timestamp = mktime(&timeinfo);
        snprintf(value_str, sizeof(value_str), "%ld", gpsInfo.timestamp);
        write_to_file("timestamp", value_str);
    }
    else
    {
        // Write empty time files
        write_to_file("year", "N/A");
        write_to_file("month", "N/A");
        write_to_file("day", "N/A");
        write_to_file("hour", "N/A");
        write_to_file("minute", "N/A");
        write_to_file("second", "N/A");
        write_to_file("millisecond", "N/A");
        write_to_file("datetime", "N/A");
        write_to_file("timestamp", "N/A");
    }

    // Only write position/speed data if fix is valid
    if (is_fix_valid)
    {
        // Calculate vertical speed and 3D speed
        calculate_vertical_speed();
        calculate_3d_speed();

        // Write basic GPS values
        snprintf(value_str, sizeof(value_str), "%.6f", gpsInfo.longitude);
        write_to_file("longitude", value_str);

        snprintf(value_str, sizeof(value_str), "%.6f", gpsInfo.latitude);
        write_to_file("latitude", value_str);

        snprintf(value_str, sizeof(value_str), "%.1f", gpsInfo.altitude);
        write_to_file("altitude", value_str);

        snprintf(value_str, sizeof(value_str), "%d", gpsInfo.satellites);
        write_to_file("satellites", value_str);

        // Write 2D and 3D speed values
        snprintf(value_str, sizeof(value_str), "%.2f", gpsInfo.speed_2d);
        write_to_file("speed2d", value_str);

        snprintf(value_str, sizeof(value_str), "%.2f", gpsInfo.speed_3d);
        write_to_file("speed3d", value_str);

        // Write vertical speed
        snprintf(value_str, sizeof(value_str), "%.2f", gpsInfo.vertical_speed);
        write_to_file("vertical_speed", value_str);

        // Write DOP values
        snprintf(value_str, sizeof(value_str), "%.2f", gpsInfo.pdop);
        write_to_file("pdop", value_str);

        snprintf(value_str, sizeof(value_str), "%.2f", gpsInfo.hdop);
        write_to_file("hdop", value_str);

        snprintf(value_str, sizeof(value_str), "%.2f", gpsInfo.vdop);
        write_to_file("vdop", value_str);
    }
    else
    {
        // Write empty files for invalid data
        write_to_file("longitude", "N/A");
        write_to_file("latitude", "N/A");
        write_to_file("altitude", "N/A");
        write_to_file("satellites", "N/A");
        write_to_file("speed2d", "N/A");
        write_to_file("speed3d", "N/A");
        write_to_file("vertical_speed", "N/A");
        write_to_file("pdop", "N/A");
        write_to_file("hdop", "N/A");
        write_to_file("vdop", "N/A");
    }
}

// Function to parse the NMEA sentence
void parse_nmea_sentence(const char* sentence)
{
    if (sentence[0] != '$')
        return;

    // Print all raw sentences
    printf("%s\n", sentence);

    // Only process sentences we care about to save CPU
    if (strncmp(sentence, "$GPRMC", 6) != 0 &&
        strncmp(sentence, "$GPVTG", 6) != 0 &&
        strncmp(sentence, "$GPGGA", 6) != 0 &&
        strncmp(sentence, "$GPGLL", 6) != 0 &&
        strncmp(sentence, "$GPGSA", 6) != 0)
    {
        // Print unknown sentence type
        printf("Unknown sentence type: %s\n", sentence);
        return;
    }

    // Make a copy since split_string modifies the string
    char sentence_copy[256];
    strncpy(sentence_copy, sentence, sizeof(sentence_copy));
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';

    char* components[20];
    int num_components = split_string(sentence_copy, components, 20);

    if (strncmp(sentence, "$GPRMC", 6) == 0)
    {
        if (num_components >= 10)
        {
            // Check if time field is not empty
            if (strlen(components[1]) > 0)
            {
                sscanf(components[1], "%2d%2d%2d.%2d", &gpsInfo.hour, &gpsInfo.minute, &gpsInfo.second, &gpsInfo.millisecond);
                gpsInfo.has_time = 1;

                float latitude = atof(components[3]);
                char lat_dir = components[4][0];
                float longitude = atof(components[5]);
                char lon_dir = components[6][0];
                gpsInfo.latitude = convert_to_decimal_degrees(latitude, lat_dir);
                gpsInfo.longitude = convert_to_decimal_degrees(longitude, lon_dir);
                gpsInfo.knots = atof(components[7]);

                // Check if date field is not empty
                if (strlen(components[9]) > 0)
                {
                    int date = atoi(components[9]);
                    gpsInfo.day = date / 10000;
                    gpsInfo.month = (date / 100) % 100;
                    gpsInfo.year = date % 100 + 2000;
                }

                // Check if data is valid (status field)
                if (components[2][0] == 'A')
                {
                    gpsInfo.valid_fix = 1;
                }
                else
                {
                    gpsInfo.valid_fix = 0;
                }
            }
            else
            {
                gpsInfo.has_time = 0;
                gpsInfo.valid_fix = 0;
            }

            update_gps_files();
        }
    }
    else if (strncmp(sentence, "$GPVTG", 6) == 0)
    {
        if (num_components >= 8)
        {
            gpsInfo.knots = atof(components[5]);
            gpsInfo.speed_2d = atof(components[7]); // 2D speed in km/h

            update_gps_files();
        }
    }
    else if (strncmp(sentence, "$GPGGA", 6) == 0)
    {
        if (num_components >= 9)
        {
            // Check if time field is not empty
            if (strlen(components[1]) > 0)
            {
                sscanf(components[1], "%2d%2d%2d.%2d", &gpsInfo.hour, &gpsInfo.minute, &gpsInfo.second, &gpsInfo.millisecond);
                gpsInfo.has_time = 1;

                float latitude = atof(components[2]);
                char lat_dir = components[3][0];
                float longitude = atof(components[4]);
                char lon_dir = components[5][0];
                gpsInfo.latitude = convert_to_decimal_degrees(latitude, lat_dir);
                gpsInfo.longitude = convert_to_decimal_degrees(longitude, lon_dir);
                gpsInfo.satellites = atoi(components[7]);
                gpsInfo.altitude = atof(components[9]);

                // Check if data is valid (fix quality field)
                if (atoi(components[6]) > 0)
                {
                    gpsInfo.valid_fix = 1;
                }
                else
                {
                    gpsInfo.valid_fix = 0;
                }
            }
            else
            {
                gpsInfo.has_time = 0;
                gpsInfo.valid_fix = 0;
            }

            update_gps_files();
        }
    }
    else if (strncmp(sentence, "$GPGLL", 6) == 0)
    {
        if (num_components >= 6)
        {
            // Check if time field is not empty
            if (strlen(components[5]) > 0)
            {
                sscanf(components[5], "%2d%2d%2d.%2d", &gpsInfo.hour, &gpsInfo.minute, &gpsInfo.second, &gpsInfo.millisecond);
                gpsInfo.has_time = 1;

                float latitude = atof(components[1]);
                char lat_dir = components[2][0];
                float longitude = atof(components[3]);
                char lon_dir = components[4][0];
                gpsInfo.latitude = convert_to_decimal_degrees(latitude, lat_dir);
                gpsInfo.longitude = convert_to_decimal_degrees(longitude, lon_dir);

                // Check if data is valid (status field)
                if (components[6][0] == 'A')
                {
                    gpsInfo.valid_fix = 1;
                }
                else
                {
                    gpsInfo.valid_fix = 0;
                }
            }
            else
            {
                gpsInfo.has_time = 0;
                gpsInfo.valid_fix = 0;
            }

            update_gps_files();
        }
    }
    else if (strncmp(sentence, "$GPGSA", 6) == 0)
    {
        // $GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
        if (num_components >= 18)
        {
            // Parse DOP values (last three fields)
            gpsInfo.pdop = atof(components[15]); // Position Dilution of Precision
            gpsInfo.hdop = atof(components[16]); // Horizontal Dilution of Precision
            gpsInfo.vdop = atof(components[17]); // Vertical Dilution of Precision

            update_gps_files();
        }
    }
}

void print_usage(const char* program_name)
{
    printf("Usage: %s [options]\n", program_name);
    printf("Options:\n");
    printf("  -p <port>     Serial port (default: %s)\n", DEFAULT_GPS_PORT);
    printf("  -b <baudrate> Baud rate (default: 9600)\n");
    printf("  -h            Show this help message\n");
}

int main(int argc, char* argv[])
{
    char* gps_port = DEFAULT_GPS_PORT;
    int baudrate = 9600;
    speed_t speed = DEFAULT_BAUDRATE;

    // Parse command line arguments
    int opt;
    while ((opt = getopt(argc, argv, "p:b:h")) != -1)
    {
        switch (opt)
        {
        case 'p':
            gps_port = optarg;
            break;
        case 'b':
            baudrate = atoi(optarg);
            speed = get_baudrate(baudrate);
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("Using serial port: %s, baudrate: %d\n", gps_port, baudrate);

    // Initialize GPS data structure
    memset(&gpsInfo, 0, sizeof(gpsInfo));
    gpsInfo.valid_time = 0; // Initially invalid time
    gpsInfo.valid_fix = 0;  // Initially invalid fix
    gpsInfo.has_time = 0;   // Initially no time data

    // Create /tmp/gps directory
    mkdir(GPS_DIR, 0777);

    // Write initial validity files
    write_to_file("valid_time", "0");
    write_to_file("valid_fix", "0");

    int serial_fd = open(gps_port, O_RDWR | O_NOCTTY);
    if (serial_fd == -1)
    {
        perror("Error opening serial port");
        return 1;
    }

    // Configure the serial port
    if (configure_serial_port(serial_fd, speed) != 0)
    {
        close(serial_fd);
        return 1;
    }

    // Set non-blocking mode to avoid busy waiting
    fcntl(serial_fd, F_SETFL, O_NONBLOCK);

    char buffer[256];
    while (1)
    {
        if (read_line(serial_fd, buffer, sizeof(buffer)) == 0)
        {
            if (strlen(buffer) > 0)
            {
                parse_nmea_sentence(buffer);
            }
        }
        usleep(10000); // Sleep for 10ms to reduce CPU usage
    }

    close(serial_fd);
    return 0;
}