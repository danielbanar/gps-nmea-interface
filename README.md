# GPS Interface & Monitor

This project provides apps for interfacing GPS modules connected over a
serial port.

## Features

-   Supports configurable serial port and baud rate.
-   Parses common NMEA sentences: `$GPRMC`, `$GPVTG`, `$GPGGA`,
    `$GPGLL`, `$GPGSA`.
-   Converts latitude/longitude from NMEA to decimal degrees.
-   Logs GPS time, location, altitude, speed (2D/3D), vertical speed,
    and DOP values.
-   Uses `/tmp/gps/gps_data` as the central file for parsed GPS data.

This is the **low CPU usage branch**. The **separate branch** writes
every variable (such as altitude, day, minute, longitude, fix) into
separate files under `/tmp/gps/`.

## Building

Simply run:

``` bash
make
```

## Usage

### Running the GPS interface

``` bash
./gps [options]
```

**Options:**
- `-p <port>` -- Serial port (default: `/dev/ttyS0`)
- `-b <baudrate>` -- Baud rate (default: `9600`)
- `-v` -- Enable verbose mode (prints raw NMEA sentences) 
- `-h` -- Show help

Example:

``` bash
./gps -p /dev/ttyUSB0 -b 115200 -v
```

This will start reading GPS data and store structured output in
`/tmp/gps/gps_data`.

### Running the GPS monitor

In another terminal, run:

``` bash
./gps_monitor
```

This will display the GPS data in a formatted, continuously updating
view.

## Example Output

    ╔══════════════════════════════════════════════════════════════════════════════╗
    ║                           GPS DATA MONITOR                                          ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║ Status: FIX                                                                         ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║ Date/Time:                                                                          ║
    ║   Combined: 2025-08-30 12:34:56                                                     ║
    ║   Date: 2025-08-30                                                                  ║
    ║   Time: 12:34:56                                                                    ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║ Location:                                                                           ║
    ║   Latitude: 12.345678° N                                                            ║
    ║   Longitude: 12.345678° E                                                           ║
    ║   Altitude: 30.5 m                                                                  ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║ Speed:                                                                              ║
    ║   2D: 5.67 km/h                                                                     ║
    ║   3D: 5.70 km/h                                                                     ║
    ║   Vertical: 0.10 m/s                                                                ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║ Satellites: 7                                                                       ║
    ║ DOP: PDOP: 1.23, HDOP: 0.98, VDOP: 1.11                                             ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║ Timestamp: 1693407296                                                               ║
    ╚══════════════════════════════════════════════════════════════════════════════╝

------------------------------------------------------------------------
