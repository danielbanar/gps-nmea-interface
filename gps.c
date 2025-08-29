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

#define GPS_PORT "/dev/ttyS0"
#define BAUDRATE B9600
#define GPS_DIR "/tmp/gps"

// Function to configure the serial port
int configure_serial_port(int fd)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0)
    {
        perror("Error getting serial port attributes");
        return -1;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

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
    int valid;                 // Flag to indicate if GPS data is valid
    double vertical_speed;     // Vertical speed in m/s
    double last_altitude;      // Previous altitude for vertical speed calculation
    time_t last_altitude_time; // Time of last altitude measurement
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

    // Only update files if we have valid data
    if (gpsInfo.year < 2000)
    {
        write_to_file("valid", "0");
        return;
    }

    // Calculate vertical speed and 3D speed
    calculate_vertical_speed();
    calculate_3d_speed();

    // Write validity flag
    write_to_file("valid", "1");

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

// Function to parse the NMEA sentence
void parse_nmea_sentence(const char* sentence)
{
    if (sentence[0] != '$')
        return;

    // Only process sentences we care about to save CPU
    if (strncmp(sentence, "$GPRMC", 6) != 0 &&
        strncmp(sentence, "$GPVTG", 6) != 0 &&
        strncmp(sentence, "$GPGGA", 6) != 0 &&
        strncmp(sentence, "$GPGLL", 6) != 0)
    {
        return;
    }

    // Print only the raw sentence
    printf("%s\n", sentence);

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
            sscanf(components[1], "%2d%2d%2d.%2d", &gpsInfo.hour, &gpsInfo.minute, &gpsInfo.second, &gpsInfo.millisecond);
            float latitude = atof(components[3]);
            char lat_dir = components[4][0];
            float longitude = atof(components[5]);
            char lon_dir = components[6][0];
            gpsInfo.latitude = convert_to_decimal_degrees(latitude, lat_dir);
            gpsInfo.longitude = convert_to_decimal_degrees(longitude, lon_dir);
            gpsInfo.knots = atof(components[7]);
            int date = atoi(components[9]);
            gpsInfo.day = date / 10000;
            gpsInfo.month = (date / 100) % 100;
            gpsInfo.year = date % 100 + 2000;

            // Check if data is valid (status field)
            if (components[2][0] == 'A')
            {
                gpsInfo.valid = 1;
            }
            else
            {
                gpsInfo.valid = 0;
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
            sscanf(components[1], "%2d%2d%2d.%2d", &gpsInfo.hour, &gpsInfo.minute, &gpsInfo.second, &gpsInfo.millisecond);
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
                gpsInfo.valid = 1;
            }
            else
            {
                gpsInfo.valid = 0;
            }

            update_gps_files();
        }
    }
    else if (strncmp(sentence, "$GPGLL", 6) == 0)
    {
        if (num_components >= 6)
        {
            float latitude = atof(components[1]);
            char lat_dir = components[2][0];
            float longitude = atof(components[3]);
            char lon_dir = components[4][0];
            gpsInfo.latitude = convert_to_decimal_degrees(latitude, lat_dir);
            gpsInfo.longitude = convert_to_decimal_degrees(longitude, lon_dir);
            sscanf(components[5], "%2d%2d%2d.%2d", &gpsInfo.hour, &gpsInfo.minute, &gpsInfo.second, &gpsInfo.millisecond);

            // Check if data is valid (status field)
            if (components[6][0] == 'A')
            {
                gpsInfo.valid = 1;
            }
            else
            {
                gpsInfo.valid = 0;
            }

            update_gps_files();
        }
    }
}

int main()
{
    // Initialize GPS data structure
    memset(&gpsInfo, 0, sizeof(gpsInfo));
    gpsInfo.valid = 0; // Initially invalid

    // Create /tmp/gps directory
    mkdir(GPS_DIR, 0777);

    // Write initial valid file
    write_to_file("valid", "0");

    int serial_fd = open(GPS_PORT, O_RDWR | O_NOCTTY);
    if (serial_fd == -1)
    {
        perror("Error opening serial port");
        return 1;
    }

    // Configure the serial port
    if (configure_serial_port(serial_fd) != 0)
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