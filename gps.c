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
#define GPS_DATA_FILE "/tmp/gps/gps_data"
#define MAX_SATELLITES 24    // Maximum number of satellites to track
#define DEFAULT_WAIT_TIME 10 // Default wait time in milliseconds

// Global flags
int verbose = 0;
int wait_time_ms = DEFAULT_WAIT_TIME;

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
                usleep(wait_time_ms * 1000); // Use the configured wait time
                continue;
            }
            perror("Error reading from serial port");
            return -1;
        }
        if (bytes_read == 0)
        {
            usleep(wait_time_ms * 1000); // Use the configured wait time
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

// Structure to store information about individual satellites
struct SatelliteInfo
{
    int prn;       // Satellite PRN number
    int elevation; // Elevation in degrees
    int azimuth;   // Azimuth in degrees
    int snr;       // Signal-to-Noise Ratio in dBHz
};

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
    char status[16];           // Status string
    char datetime[64];         // Formatted date and time

    // GSV sentence data
    int satellites_in_view;                              // Total number of satellites in view
    struct SatelliteInfo satellite_info[MAX_SATELLITES]; // Info for each satellite
};

struct GPSData gpsInfo;

// Function to write all GPS data to a single file
void write_gps_data()
{
    FILE* file = fopen(GPS_DATA_FILE, "w");
    if (file)
    {
        // Write status
        fprintf(file, "status: %s\n", gpsInfo.status);

        // Write time data if available
        if (gpsInfo.valid_time)
        {
            fprintf(file, "datetime: %s\n", gpsInfo.datetime);
            fprintf(file, "timestamp: %ld\n", gpsInfo.timestamp);
            fprintf(file, "year: %d\n", gpsInfo.year);
            fprintf(file, "month: %d\n", gpsInfo.month);
            fprintf(file, "day: %d\n", gpsInfo.day);
            fprintf(file, "hour: %d\n", gpsInfo.hour);
            fprintf(file, "minute: %d\n", gpsInfo.minute);
            fprintf(file, "second: %d\n", gpsInfo.second);
            fprintf(file, "millisecond: %d\n", gpsInfo.millisecond);
        }
        else
        {
            fprintf(file, "datetime: N/A\n");
            fprintf(file, "timestamp: N/A\n");
            fprintf(file, "year: N/A\n");
            fprintf(file, "month: N/A\n");
            fprintf(file, "day: N/A\n");
            fprintf(file, "hour: N/A\n");
            fprintf(file, "minute: N/A\n");
            fprintf(file, "second: N/A\n");
            fprintf(file, "millisecond: N/A\n");
        }

        // Write position/speed data if fix is valid
        if (gpsInfo.valid_fix)
        {
            fprintf(file, "latitude: %.6f\n", gpsInfo.latitude);
            fprintf(file, "longitude: %.6f\n", gpsInfo.longitude);
            fprintf(file, "altitude: %.1f\n", gpsInfo.altitude);
            fprintf(file, "satellites: %d\n", gpsInfo.satellites);
            fprintf(file, "speed2d: %.2f\n", gpsInfo.speed_2d);
            fprintf(file, "speed3d: %.2f\n", gpsInfo.speed_3d);
            fprintf(file, "vertical_speed: %.2f\n", gpsInfo.vertical_speed);
            fprintf(file, "pdop: %.2f\n", gpsInfo.pdop);
            fprintf(file, "hdop: %.2f\n", gpsInfo.hdop);
            fprintf(file, "vdop: %.2f\n", gpsInfo.vdop);
        }
        else
        {
            fprintf(file, "latitude: N/A\n");
            fprintf(file, "longitude: N/A\n");
            fprintf(file, "altitude: N/A\n");
            fprintf(file, "satellites: 0\n");
            fprintf(file, "speed2d: N/A\n");
            fprintf(file, "speed3d: N/A\n");
            fprintf(file, "vertical_speed: N/A\n");
            fprintf(file, "pdop: N/A\n");
            fprintf(file, "hdop: N/A\n");
            fprintf(file, "vdop: N/A\n");
        }

        // Write satellite information
        fprintf(file, "satellites_in_view: %d\n", gpsInfo.satellites_in_view);
        for (int i = 0; i < gpsInfo.satellites_in_view && i < MAX_SATELLITES; i++)
        {
            fprintf(file, "satellite_%d_prn: %d\n", i + 1, gpsInfo.satellite_info[i].prn);
            fprintf(file, "satellite_%d_elevation: %d\n", i + 1, gpsInfo.satellite_info[i].elevation);
            fprintf(file, "satellite_%d_azimuth: %d\n", i + 1, gpsInfo.satellite_info[i].azimuth);
            fprintf(file, "satellite_%d_snr: %d\n", i + 1, gpsInfo.satellite_info[i].snr);
        }

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

// Function to update GPS data and write to file
void update_gps_data()
{
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

    // Set status string
    if (is_fix_valid)
    {
        strcpy(gpsInfo.status, "FIX");
    }
    else if (is_time_valid)
    {
        strcpy(gpsInfo.status, "NO_FIX");
    }
    else
    {
        strcpy(gpsInfo.status, "NO_TIME");
    }

    // Format date and time if available
    if (is_time_valid)
    {
        snprintf(gpsInfo.datetime, sizeof(gpsInfo.datetime), "%04d-%02d-%02d %02d:%02d:%02d",
                 gpsInfo.year, gpsInfo.month, gpsInfo.day,
                 gpsInfo.hour, gpsInfo.minute, gpsInfo.second);

        // Calculate timestamp
        timeinfo.tm_year = gpsInfo.year - 1900;
        timeinfo.tm_mon = gpsInfo.month - 1;
        timeinfo.tm_mday = gpsInfo.day;
        timeinfo.tm_hour = gpsInfo.hour;
        timeinfo.tm_min = gpsInfo.minute;
        timeinfo.tm_sec = gpsInfo.second;
        timeinfo.tm_isdst = -1;

        gpsInfo.timestamp = mktime(&timeinfo);
    }
    else
    {
        strcpy(gpsInfo.datetime, "N/A");
        gpsInfo.timestamp = 0;
    }

    // Calculate speeds if fix is valid
    if (is_fix_valid)
    {
        calculate_vertical_speed();
        calculate_3d_speed();
    }
    else
    {
        gpsInfo.vertical_speed = 0;
        gpsInfo.speed_3d = 0;
    }

    // Write all data to the single file
    write_gps_data();
}

// Function to parse the NMEA sentence
void parse_nmea_sentence(const char* sentence)
{
    if (sentence[0] != '$')
        return;

    // Print all raw sentences only if verbose mode is enabled
    if (verbose)
    {
        printf("%s\n", sentence);
    }

    // Check for supported sentence types (both standard GPS and multi-GNSS)
    if (strncmp(sentence, "$GPRMC", 6) != 0 &&
        strncmp(sentence, "$GNRMC", 6) != 0 &&
        strncmp(sentence, "$GPVTG", 6) != 0 &&
        strncmp(sentence, "$GNVTG", 6) != 0 &&
        strncmp(sentence, "$GPGGA", 6) != 0 &&
        strncmp(sentence, "$GNGGA", 6) != 0 &&
        strncmp(sentence, "$GPGLL", 6) != 0 &&
        strncmp(sentence, "$GNGLL", 6) != 0 &&
        strncmp(sentence, "$GPGSA", 6) != 0 &&
        strncmp(sentence, "$GNGSA", 6) != 0 &&
        strncmp(sentence, "$GPGSV", 6) != 0 &&
        strncmp(sentence, "$GAGSV", 6) != 0 &&
        strncmp(sentence, "$GBGSV", 6) != 0 &&
        strncmp(sentence, "$GQGSV", 6) != 0 &&
        strncmp(sentence, "$GLGSV", 6) != 0)
    {
        // Print unknown sentence type only if verbose mode is enabled
        if (verbose)
        {
            printf("Unknown sentence type: %s\n", sentence);
        }
        return;
    }

    // Make a copy since split_string modifies the string
    char sentence_copy[256];
    strncpy(sentence_copy, sentence, sizeof(sentence_copy));
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';

    char* components[20];
    int num_components = split_string(sentence_copy, components, 20);

    // Handle RMC sentences (both GP and GN)
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
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

            update_gps_data();
        }
    }
    // Handle VTG sentences (both GP and GN)
    else if (strncmp(sentence, "$GPVTG", 6) == 0 || strncmp(sentence, "$GNVTG", 6) == 0)
    {
        if (num_components >= 8)
        {
            gpsInfo.knots = atof(components[5]);
            gpsInfo.speed_2d = atof(components[7]); // 2D speed in km/h

            update_gps_data();
        }
    }
    // Handle GGA sentences (both GP and GN)
    else if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0)
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

            update_gps_data();
        }
    }
    // Handle GLL sentences (both GP and GN)
    else if (strncmp(sentence, "$GPGLL", 6) == 0 || strncmp(sentence, "$GNGLL", 6) == 0)
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

            update_gps_data();
        }
    }
    // Handle GSA sentences (both GP and GN)
    else if (strncmp(sentence, "$GPGSA", 6) == 0 || strncmp(sentence, "$GNGSA", 6) == 0)
    {
        // $GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
        if (num_components >= 18)
        {
            // Parse DOP values (last three fields)
            gpsInfo.pdop = atof(components[15]); // Position Dilution of Precision
            gpsInfo.hdop = atof(components[16]); // Horizontal Dilution of Precision
            gpsInfo.vdop = atof(components[17]); // Vertical Dilution of Precision

            update_gps_data();
        }
    }
    // Handle GSV sentences from all supported satellite systems
    else if (strncmp(sentence, "$GPGSV", 6) == 0 ||
             strncmp(sentence, "$GAGSV", 6) == 0 || // Galileo
             strncmp(sentence, "$GBGSV", 6) == 0 || // BeiDou
             strncmp(sentence, "$GQGSV", 6) == 0 || // QZSS
             strncmp(sentence, "$GLGSV", 6) == 0)   // GLONASS
    {
        // $GPGSV,4,4,14,25,10,246,27,32,06,325,*7C
        if (num_components >= 8)
        {
            int total_messages = atoi(components[1]);     // Total number of GSV messages
            int message_num = atoi(components[2]);        // Current message number
            int satellites_in_view = atoi(components[3]); // Total satellites in view

            // Update the total satellites in view if this is higher than current count
            if (satellites_in_view > gpsInfo.satellites_in_view)
            {
                gpsInfo.satellites_in_view = satellites_in_view;
            }

            // Calculate how many satellites are described in this message
            int satellites_in_this_message = (num_components - 4) / 4;

            // Start index for satellite data in the array
            int start_index = (message_num - 1) * 4;

            // Parse satellite data
            for (int i = 0; i < satellites_in_this_message; i++)
            {
                int base_index = 4 + (i * 4);
                int sat_index = start_index + i;

                if (sat_index < MAX_SATELLITES)
                {
                    gpsInfo.satellite_info[sat_index].prn = atoi(components[base_index]);
                    gpsInfo.satellite_info[sat_index].elevation = atoi(components[base_index + 1]);
                    gpsInfo.satellite_info[sat_index].azimuth = atoi(components[base_index + 2]);

                    // Handle empty SNR values (represented by empty string)
                    if (strlen(components[base_index + 3]) > 0)
                    {
                        gpsInfo.satellite_info[sat_index].snr = atoi(components[base_index + 3]);
                    }
                    else
                    {
                        gpsInfo.satellite_info[sat_index].snr = 0; // No signal
                    }
                }
            }

            update_gps_data();
        }
    }
}

void print_usage(const char* program_name)
{
    printf("Usage: %s [options]\n", program_name);
    printf("Options:\n");
    printf("  -p <port>     Serial port (default: %s)\n", DEFAULT_GPS_PORT);
    printf("  -b <baudrate> Baud rate (default: 9600)\n");
    printf("  -w <ms>       Wait time in milliseconds (default: %d)\n", DEFAULT_WAIT_TIME);
    printf("  -v            Enable verbose output (print NMEA sentences)\n");
    printf("  -h            Show this help message\n");
}

int main(int argc, char* argv[])
{
    char* gps_port = DEFAULT_GPS_PORT;
    int baudrate = 9600;
    speed_t speed = DEFAULT_BAUDRATE;
    verbose = 0;                      // Default to non-verbose mode
    wait_time_ms = DEFAULT_WAIT_TIME; // Default wait time

    // Parse command line arguments
    int opt;
    while ((opt = getopt(argc, argv, "p:b:w:vh")) != -1)
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
        case 'w':
            wait_time_ms = atoi(optarg);
            if (wait_time_ms < 1)
                wait_time_ms = 1; // Minimum 1ms
            if (wait_time_ms > 1000)
                wait_time_ms = 1000; // Maximum 1000ms
            break;
        case 'v':
            verbose = 1;
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("Using serial port: %s, baudrate: %d, wait time: %dms\n",
           gps_port, baudrate, wait_time_ms);
    if (verbose)
    {
        printf("Verbose mode enabled - printing NMEA sentences\n");
    }

    // Initialize GPS data structure
    memset(&gpsInfo, 0, sizeof(gpsInfo));
    gpsInfo.valid_time = 0;         // Initially invalid time
    gpsInfo.valid_fix = 0;          // Initially invalid fix
    gpsInfo.has_time = 0;           // Initially no time data
    gpsInfo.satellites_in_view = 0; // Initially no satellites in view
    strcpy(gpsInfo.status, "NO_TIME");
    strcpy(gpsInfo.datetime, "N/A");

    // Create /tmp/gps directory
    mkdir(GPS_DIR, 0777);

    // Write initial data file
    write_gps_data();

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
        usleep(wait_time_ms * 1000); // Use the configured wait time
    }

    close(serial_fd);
    return 0;
}