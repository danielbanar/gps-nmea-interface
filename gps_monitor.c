#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define GPS_DIR "/tmp/gps"

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

void display_gps_data()
{
    char value[64];
    int valid = 0;

    // Read validity flag
    if (read_from_file("valid", value, sizeof(value)))
    {
        valid = atoi(value);
    }

    if (!valid)
    {
        printf("GPS Data: No valid fix\n");
        return;
    }

    // Clear screen
    printf("\033[2J\033[H");

    printf("=== GPS DATA MONITOR ===\n\n");

    // Read and display date/time
    if (read_from_file("datetime", value, sizeof(value)))
    {
        printf("Date/Time: %s\n", value);
    }

    // Read and display location
    char lon[64], lat[64], alt[64];
    if (read_from_file("longitude", lon, sizeof(lon)) &&
        read_from_file("latitude", lat, sizeof(lat)) &&
        read_from_file("altitude", alt, sizeof(alt)))
    {
        printf("Location: Lat %s, Lon %s, Alt %s m\n", lat, lon, alt);
    }

    // Read and display speed
    char speed2d[64], speed3d[64], vspeed[64];
    if (read_from_file("speed2d", speed2d, sizeof(speed2d)) &&
        read_from_file("speed3d", speed3d, sizeof(speed3d)) &&
        read_from_file("vertical_speed", vspeed, sizeof(vspeed)))
    {
        printf("Speed: 2D: %s km/h, 3D: %s km/h, Vertical: %s m/s\n", speed2d, speed3d, vspeed);
    }

    // Read and display satellites
    if (read_from_file("satellites", value, sizeof(value)))
    {
        printf("Satellites: %s\n", value);
    }

    // Read and display timestamp
    if (read_from_file("timestamp", value, sizeof(value)))
    {
        printf("Timestamp: %s\n", value);
    }

    printf("\nPress Ctrl+C to exit\n");
}

int main()
{
    // Check if /tmp/gps directory exists
    struct stat st;
    if (stat(GPS_DIR, &st) == -1)
    {
        printf("GPS data directory not found. Please run gps_reader first.\n");
        return 1;
    }

    while (1)
    {
        display_gps_data();
        sleep(1); // Update every second
    }

    return 0;
}