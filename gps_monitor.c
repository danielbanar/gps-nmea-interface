#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define GPS_DATA_FILE "/tmp/gps/gps_data"
#define SCREEN_WIDTH 80

// Structure to hold parsed GPS data
struct GPSDisplayData
{
    char status[16];
    char datetime[64];
    char timestamp[32];
    char year[8];
    char month[8];
    char day[8];
    char hour[8];
    char minute[8];
    char second[8];
    char millisecond[8];
    char latitude[32];
    char longitude[32];
    char altitude[32];
    char satellites[8];
    char speed2d[32];
    char speed3d[32];
    char vertical_speed[32];
    char pdop[32];
    char hdop[32];
    char vdop[32];
};

// Function to parse the GPS data file
int parse_gps_data_file(struct GPSDisplayData* data)
{
    FILE* file = fopen(GPS_DATA_FILE, "r");
    if (!file)
    {
        return 0;
    }

    // Initialize all fields to "N/A"
    strcpy(data->status, "N/A");
    strcpy(data->datetime, "N/A");
    strcpy(data->timestamp, "N/A");
    strcpy(data->year, "N/A");
    strcpy(data->month, "N/A");
    strcpy(data->day, "N/A");
    strcpy(data->hour, "N/A");
    strcpy(data->minute, "N/A");
    strcpy(data->second, "N/A");
    strcpy(data->millisecond, "N/A");
    strcpy(data->latitude, "N/A");
    strcpy(data->longitude, "N/A");
    strcpy(data->altitude, "N/A");
    strcpy(data->satellites, "0");
    strcpy(data->speed2d, "N/A");
    strcpy(data->speed3d, "N/A");
    strcpy(data->vertical_speed, "N/A");
    strcpy(data->pdop, "N/A");
    strcpy(data->hdop, "N/A");
    strcpy(data->vdop, "N/A");

    char line[128];
    while (fgets(line, sizeof(line), file) != NULL)
    {
        // Remove newline character if present
        size_t len = strlen(line);
        if (len > 0 && line[len - 1] == '\n')
        {
            line[len - 1] = '\0';
        }

        // Parse key-value pairs
        char* key = strtok(line, ":");
        char* value = strtok(NULL, ":");

        if (key && value)
        {
            // Skip leading space in value
            if (value[0] == ' ')
                value++;

            if (strcmp(key, "status") == 0)
            {
                strncpy(data->status, value, sizeof(data->status) - 1);
            }
            else if (strcmp(key, "datetime") == 0)
            {
                strncpy(data->datetime, value, sizeof(data->datetime) - 1);
            }
            else if (strcmp(key, "timestamp") == 0)
            {
                strncpy(data->timestamp, value, sizeof(data->timestamp) - 1);
            }
            else if (strcmp(key, "year") == 0)
            {
                strncpy(data->year, value, sizeof(data->year) - 1);
            }
            else if (strcmp(key, "month") == 0)
            {
                strncpy(data->month, value, sizeof(data->month) - 1);
            }
            else if (strcmp(key, "day") == 0)
            {
                strncpy(data->day, value, sizeof(data->day) - 1);
            }
            else if (strcmp(key, "hour") == 0)
            {
                strncpy(data->hour, value, sizeof(data->hour) - 1);
            }
            else if (strcmp(key, "minute") == 0)
            {
                strncpy(data->minute, value, sizeof(data->minute) - 1);
            }
            else if (strcmp(key, "second") == 0)
            {
                strncpy(data->second, value, sizeof(data->second) - 1);
            }
            else if (strcmp(key, "millisecond") == 0)
            {
                strncpy(data->millisecond, value, sizeof(data->millisecond) - 1);
            }
            else if (strcmp(key, "latitude") == 0)
            {
                strncpy(data->latitude, value, sizeof(data->latitude) - 1);
            }
            else if (strcmp(key, "longitude") == 0)
            {
                strncpy(data->longitude, value, sizeof(data->longitude) - 1);
            }
            else if (strcmp(key, "altitude") == 0)
            {
                strncpy(data->altitude, value, sizeof(data->altitude) - 1);
            }
            else if (strcmp(key, "satellites") == 0)
            {
                strncpy(data->satellites, value, sizeof(data->satellites) - 1);
            }
            else if (strcmp(key, "speed2d") == 0)
            {
                strncpy(data->speed2d, value, sizeof(data->speed2d) - 1);
            }
            else if (strcmp(key, "speed3d") == 0)
            {
                strncpy(data->speed3d, value, sizeof(data->speed3d) - 1);
            }
            else if (strcmp(key, "vertical_speed") == 0)
            {
                strncpy(data->vertical_speed, value, sizeof(data->vertical_speed) - 1);
            }
            else if (strcmp(key, "pdop") == 0)
            {
                strncpy(data->pdop, value, sizeof(data->pdop) - 1);
            }
            else if (strcmp(key, "hdop") == 0)
            {
                strncpy(data->hdop, value, sizeof(data->hdop) - 1);
            }
            else if (strcmp(key, "vdop") == 0)
            {
                strncpy(data->vdop, value, sizeof(data->vdop) - 1);
            }
        }
    }

    fclose(file);
    return 1;
}

// Function to create a formatted line with proper padding
void format_line(const char* content)
{
    printf("║ %-76s ║\n", content);
}

// Function to format time components with leading zeros
void format_time_component(char* dest, size_t dest_size, const char* value, int max_digits)
{
    if (strcmp(value, "N/A") == 0)
    {
        strncpy(dest, "N/A", dest_size);
        return;
    }

    int num = atoi(value);
    if (max_digits == 2)
    {
        snprintf(dest, dest_size, "%02d", num);
    }
    else
    {
        snprintf(dest, dest_size, "%d", num);
    }
}

// Function to reconstruct the combined datetime string
void reconstruct_datetime(char* dest, size_t dest_size, const char* year, const char* month,
                          const char* day, const char* hour, const char* minute, const char* second)
{
    if (strcmp(year, "N/A") == 0 || strcmp(month, "N/A") == 0 || strcmp(day, "N/A") == 0 ||
        strcmp(hour, "N/A") == 0 || strcmp(minute, "N/A") == 0 || strcmp(second, "N/A") == 0)
    {
        strncpy(dest, "N/A", dest_size);
        return;
    }

    snprintf(dest, dest_size, "%s-%s-%s %s:%s:%s",
             year, month, day, hour, minute, second);
}

// Function to format coordinates with directional letters
void format_coordinate(char* dest, size_t dest_size, const char* coord_str, int is_latitude)
{
    if (strcmp(coord_str, "N/A") == 0)
    {
        strncpy(dest, "N/A", dest_size);
        return;
    }

    double coord = atof(coord_str);
    char direction;

    if (is_latitude)
    {
        direction = (coord >= 0) ? 'N' : 'S';
        coord = fabs(coord);
        snprintf(dest, dest_size, "%.6f° %c", coord, direction);
    }
    else
    {
        direction = (coord >= 0) ? 'E' : 'W';
        coord = fabs(coord);
        snprintf(dest, dest_size, "%.6f° %c", coord, direction);
    }
}

// Function to display data in a properly aligned box
void display_boxed_data()
{
    struct GPSDisplayData data;
    if (!parse_gps_data_file(&data))
    {
        printf("Failed to read GPS data file.\n");
        return;
    }

    // Format time components with leading zeros
    char formatted_minute[8], formatted_second[8], formatted_month[8], formatted_day[8];
    format_time_component(formatted_minute, sizeof(formatted_minute), data.minute, 2);
    format_time_component(formatted_second, sizeof(formatted_second), data.second, 2);
    format_time_component(formatted_month, sizeof(formatted_month), data.month, 2);
    format_time_component(formatted_day, sizeof(formatted_day), data.day, 2);

    // Reconstruct the combined datetime string
    char reconstructed_datetime[64];
    reconstruct_datetime(reconstructed_datetime, sizeof(reconstructed_datetime),
                         data.year, formatted_month, formatted_day,
                         data.hour, formatted_minute, formatted_second);

    // Format coordinates with directional letters
    char formatted_latitude[32], formatted_longitude[32];
    format_coordinate(formatted_latitude, sizeof(formatted_latitude), data.latitude, 1);
    format_coordinate(formatted_longitude, sizeof(formatted_longitude), data.longitude, 0);

    // Clear screen
    printf("\033[2J\033[H");

    // Top border
    printf("╔══════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                           GPS DATA MONITOR                                   ║\n");
    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Status section
    char status_line[128];
    snprintf(status_line, sizeof(status_line), "Status: %s", data.status);
    format_line(status_line);
    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Date/Time section - always show
    format_line("Date/Time:");
    char time_line[128];

    // Show reconstructed datetime
    snprintf(time_line, sizeof(time_line), "  Combined: %s", reconstructed_datetime);
    format_line(time_line);

    // Show individual components with proper formatting
    snprintf(time_line, sizeof(time_line), "  Date: %s-%s-%s", data.year, formatted_month, formatted_day);
    format_line(time_line);

    // Show time without milliseconds
    snprintf(time_line, sizeof(time_line), "  Time: %s:%s:%s",
             data.hour, formatted_minute, formatted_second);
    format_line(time_line);

    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Location section - always show
    format_line("Location:");
    char loc_line[128];

    snprintf(loc_line, sizeof(loc_line), "  Latitude: %s", formatted_latitude);
    format_line(loc_line);

    snprintf(loc_line, sizeof(loc_line), "  Longitude: %s", formatted_longitude);
    format_line(loc_line);

    snprintf(loc_line, sizeof(loc_line), "  Altitude: %s m", data.altitude);
    format_line(loc_line);

    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Speed section - always show
    format_line("Speed:");
    char speed_line[128];

    snprintf(speed_line, sizeof(speed_line), "  2D: %s km/h", data.speed2d);
    format_line(speed_line);

    snprintf(speed_line, sizeof(speed_line), "  3D: %s km/h", data.speed3d);
    format_line(speed_line);

    snprintf(speed_line, sizeof(speed_line), "  Vertical: %s m/s", data.vertical_speed);
    format_line(speed_line);

    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Satellites and DOP section - always show
    char sats_line[128];
    snprintf(sats_line, sizeof(sats_line), "Satellites: %s", data.satellites);
    format_line(sats_line);

    char dop_line[128];
    snprintf(dop_line, sizeof(dop_line), "DOP: PDOP: %s, HDOP: %s, VDOP: %s",
             data.pdop, data.hdop, data.vdop);
    format_line(dop_line);

    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Timestamp section - always show
    if (strcmp(data.timestamp, "N/A") != 0)
    {
        char ts_line[128];
        snprintf(ts_line, sizeof(ts_line), "Timestamp: %s", data.timestamp);
        format_line(ts_line);
    }

    // Bottom border
    printf("╚══════════════════════════════════════════════════════════════════════════════╝\n");
    printf("\nPress Ctrl+C to exit\n");
}

int main()
{
    // Check if GPS data file exists
    struct stat st;
    if (stat(GPS_DATA_FILE, &st) == -1)
    {
        printf("GPS data file not found. Please run gps first.\n");
        return 1;
    }

    while (1)
    {
        display_boxed_data();
        sleep(1); // Update every second
    }

    return 0;
}