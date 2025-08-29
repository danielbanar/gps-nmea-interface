#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define GPS_DIR "/tmp/gps"
#define SCREEN_WIDTH 80

// Function to read a value from a file in /tmp/gps
int read_from_file(const char* filename, char* buffer, size_t buffer_size)
{
    char path[256];
    snprintf(path, sizeof(path), "%s/%s", GPS_DIR, filename);

    FILE* file = fopen(path, "r");
    if (!file)
    {
        return 0;
    }

    if (fgets(buffer, buffer_size, file) == NULL)
    {
        fclose(file);
        return 0;
    }

    // Remove newline character if present
    size_t len = strlen(buffer);
    if (len > 0 && buffer[len - 1] == '\n')
    {
        buffer[len - 1] = '\0';
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
    char value[64];
    int valid_time = 0;
    int valid_fix = 0;

    // Read validity flags
    if (read_from_file("valid_time", value, sizeof(value)))
    {
        valid_time = atoi(value);
    }
    if (read_from_file("valid_fix", value, sizeof(value)))
    {
        valid_fix = atoi(value);
    }

    // Clear screen
    printf("\033[2J\033[H");

    // Top border
    printf("╔══════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                           GPS DATA MONITOR                                   ║\n");
    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Status section
    char status_line[128];
    snprintf(status_line, sizeof(status_line), "Status: Time: %s, Fix: %s",
             valid_time ? "VALID" : "INVALID",
             valid_fix ? "VALID" : "INVALID");
    format_line(status_line);
    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Date/Time section - always show if available
    char datetime[64] = "N/A";
    if (read_from_file("datetime", datetime, sizeof(datetime)) && strlen(datetime) > 0)
    {
        char time_line[128];
        snprintf(time_line, sizeof(time_line), "Date/Time: %s", datetime);
        format_line(time_line);
        printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");
    }

    // Location section
    char lon[64] = "N/A", lat[64] = "N/A", alt[64] = "N/A";
    if (read_from_file("longitude", lon, sizeof(lon)) &&
        read_from_file("latitude", lat, sizeof(lat)) &&
        read_from_file("altitude", alt, sizeof(alt)))
    {
        if (!is_valid_value(lon, 0) || !is_valid_value(lat, 0))
        {
            strcpy(lon, "N/A");
            strcpy(lat, "N/A");
            strcpy(alt, "N/A");
        }

        format_line("Location:");
        char loc_line[128];

        snprintf(loc_line, sizeof(loc_line), "  Latitude: %s", lat);
        format_line(loc_line);

        snprintf(loc_line, sizeof(loc_line), "  Longitude: %s", lon);
        format_line(loc_line);

        snprintf(loc_line, sizeof(loc_line), "  Altitude: %s m", alt);
        format_line(loc_line);

        printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");
    }

    // Speed section
    char speed2d[64] = "N/A", speed3d[64] = "N/A", vspeed[64] = "N/A";
    if (read_from_file("speed2d", speed2d, sizeof(speed2d)) &&
        read_from_file("speed3d", speed3d, sizeof(speed3d)) &&
        read_from_file("vertical_speed", vspeed, sizeof(vspeed)))
    {
        if (!is_valid_value(speed2d, 0) || !is_valid_value(speed3d, 0))
        {
            strcpy(speed2d, "N/A");
            strcpy(speed3d, "N/A");
            strcpy(vspeed, "N/A");
        }

        format_line("Speed:");
        char speed_line[128];

        snprintf(speed_line, sizeof(speed_line), "  2D: %s km/h", speed2d);
        format_line(speed_line);

        snprintf(speed_line, sizeof(speed_line), "  3D: %s km/h", speed3d);
        format_line(speed_line);

        snprintf(speed_line, sizeof(speed_line), "  Vertical: %s m/s", vspeed);
        format_line(speed_line);

        printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");
    }

    // Satellites and DOP section
    char sats[64] = "N/A";
    if (read_from_file("satellites", sats, sizeof(sats)))
    {
        if (!is_valid_value(sats, 0) || atoi(sats) == 0)
        {
            strcpy(sats, "N/A");
        }
        char sats_line[128];
        snprintf(sats_line, sizeof(sats_line), "Satellites: %s", sats);
        format_line(sats_line);
    }

    char pdop[64] = "N/A", hdop[64] = "N/A", vdop[64] = "N/A";
    if (read_from_file("pdop", pdop, sizeof(pdop)) &&
        read_from_file("hdop", hdop, sizeof(hdop)) &&
        read_from_file("vdop", vdop, sizeof(vdop)))
    {
        if (!is_valid_value(pdop, 1) || !is_valid_value(hdop, 1) || !is_valid_value(vdop, 1))
        {
            strcpy(pdop, "N/A");
            strcpy(hdop, "N/A");
            strcpy(vdop, "N/A");
        }

        char dop_line[128];
        snprintf(dop_line, sizeof(dop_line), "DOP: PDOP: %s, HDOP: %s, VDOP: %s", pdop, hdop, vdop);
        format_line(dop_line);
    }

    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Timestamp section
    if (read_from_file("timestamp", value, sizeof(value)))
    {
        char ts_line[128];
        snprintf(ts_line, sizeof(ts_line), "Timestamp: %s", value);
        format_line(ts_line);
    }

    // Bottom border
    printf("╚══════════════════════════════════════════════════════════════════════════════╝\n");
    printf("\nPress Ctrl+C to exit\n");
}

int main()
{
    // Check if /tmp/gps directory exists
    struct stat st;
    if (stat(GPS_DIR, &st) == -1)
    {
        printf("GPS data directory not found. Please run gps first.\n");
        return 1;
    }

    while (1)
    {
        display_boxed_data();
        sleep(1); // Update every second
    }

    return 0;
}