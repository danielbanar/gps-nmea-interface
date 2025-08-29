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
    strcpy(data->satellites, "N/A");
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

// Function to check if a value represents valid data
int is_valid_value(const char* value, int is_dop)
{
    if (value[0] == '\0' || strcmp(value, "N/A") == 0)
    {
        return 0;
    }

    double num = atof(value);
    if (is_dop)
    {
        // DOP values should be reasonable (not 99.99)
        return (num < 50.0 && num > 0.0);
    }
    else
    {
        // Other values should not be zero (or very close to zero)
        return (fabs(num) > 0.0001);
    }
}

// Function to create a formatted line with proper padding
void format_line(const char* content)
{
    printf("║ %-76s ║\n", content);
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

    // Determine validity from status
    int valid_time = (strcmp(data.status, "NO_TIME") != 0);
    int valid_fix = (strcmp(data.status, "FIX") == 0);

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

    // Date/Time section - always show if available
    if (strcmp(data.datetime, "N/A") != 0)
    {
        char time_line[128];
        snprintf(time_line, sizeof(time_line), "Date/Time: %s", data.datetime);
        format_line(time_line);
        printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");
    }

    // Location section
    if (valid_fix)
    {
        format_line("Location:");
        char loc_line[128];

        snprintf(loc_line, sizeof(loc_line), "  Latitude: %s", data.latitude);
        format_line(loc_line);

        snprintf(loc_line, sizeof(loc_line), "  Longitude: %s", data.longitude);
        format_line(loc_line);

        snprintf(loc_line, sizeof(loc_line), "  Altitude: %s m", data.altitude);
        format_line(loc_line);

        printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");
    }

    // Speed section
    if (valid_fix &&
        (is_valid_value(data.speed2d, 0) ||
         is_valid_value(data.speed3d, 0) ||
         is_valid_value(data.vertical_speed, 0)))
    {
        format_line("Speed:");
        char speed_line[128];

        snprintf(speed_line, sizeof(speed_line), "  2D: %s km/h", data.speed2d);
        format_line(speed_line);

        snprintf(speed_line, sizeof(speed_line), "  3D: %s km/h", data.speed3d);
        format_line(speed_line);

        snprintf(speed_line, sizeof(speed_line), "  Vertical: %s m/s", data.vertical_speed);
        format_line(speed_line);

        printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");
    }

    // Satellites and DOP section
    if (valid_fix && is_valid_value(data.satellites, 0))
    {
        char sats_line[128];
        snprintf(sats_line, sizeof(sats_line), "Satellites: %s", data.satellites);
        format_line(sats_line);
    }

    if (valid_fix &&
        (is_valid_value(data.pdop, 1) ||
         is_valid_value(data.hdop, 1) ||
         is_valid_value(data.vdop, 1)))
    {
        char dop_line[128];
        snprintf(dop_line, sizeof(dop_line), "DOP: PDOP: %s, HDOP: %s, VDOP: %s",
                 data.pdop, data.hdop, data.vdop);
        format_line(dop_line);
    }

    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Timestamp section
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