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

// Function to draw a horizontal separator line
void draw_separator(int width)
{
    printf("╟");
    for (int i = 0; i < width - 2; i++)
    {
        printf("─");
    }
    printf("╢\n");
}

// Function to display data in a box
void display_boxed_data()
{
    char value[64];
    int valid = 0;

    // Read validity flag
    if (read_from_file("valid", value, sizeof(value)))
    {
        valid = atoi(value);
    }

    // Clear screen
    printf("\033[2J\033[H");

    // Top border
    printf("╔══════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                              GPS DATA MONITOR                               ║\n");
    printf("╠══════════════════════════════════════════════════════════════════════════════╣\n");

    // Status section
    printf("║ Status: %-67s ║\n", valid ? "VALID FIX" : "NO VALID FIX");
    draw_separator(80);

    // Date/Time section
    if (read_from_file("datetime", value, sizeof(value)))
    {
        printf("║ Date/Time: %-65s ║\n", value);
    }
    draw_separator(80);

    // Location section
    char lon[64], lat[64], alt[64];
    if (read_from_file("longitude", lon, sizeof(lon)) &&
        read_from_file("latitude", lat, sizeof(lat)) &&
        read_from_file("altitude", alt, sizeof(alt)))
    {
        printf("║ Location:                                                                ║\n");
        printf("║   Latitude: %-30s Longitude: %-26s ║\n", lat, lon);
        printf("║   Altitude: %-65s ║\n", alt);
    }
    draw_separator(80);

    // Speed section
    char speed2d[64], speed3d[64], vspeed[64];
    if (read_from_file("speed2d", speed2d, sizeof(speed2d)) &&
        read_from_file("speed3d", speed3d, sizeof(speed3d)) &&
        read_from_file("vertical_speed", vspeed, sizeof(vspeed)))
    {
        printf("║ Speed:                                                                   ║\n");
        printf("║   2D: %-10s km/h    3D: %-10s km/h    Vertical: %-8s m/s ║\n",
               speed2d, speed3d, vspeed);
    }
    draw_separator(80);

    // Satellites and DOP section
    char sats[64], pdop[64], hdop[64], vdop[64];
    if (read_from_file("satellites", sats, sizeof(sats)))
    {
        printf("║ Satellites: %-65s ║\n", sats);
    }

    if (read_from_file("pdop", pdop, sizeof(pdop)) &&
        read_from_file("hdop", hdop, sizeof(hdop)) &&
        read_from_file("vdop", vdop, sizeof(vdop)))
    {
        printf("║ DOP: PDOP: %-8s HDOP: %-8s VDOP: %-8s                     ║\n",
               pdop, hdop, vdop);
    }
    draw_separator(80);

    // Timestamp section
    if (read_from_file("timestamp", value, sizeof(value)))
    {
        printf("║ Timestamp: %-65s ║\n", value);
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
