CC = arm-linux-gnueabihf-gcc
CFLAGS = -Os
TARGETS = gps gps_monitor

all: $(TARGETS)

gps: gps.c
	$(CC) $(CFLAGS) -o gps gps.c -lm

gps_monitor: gps_monitor.c
	$(CC) $(CFLAGS) -o gps_monitor gps_monitor.c

clean:
	rm -f $(TARGETS)

install: $(TARGETS)
	cp gps /usr/local/bin/
	cp gps_monitor /usr/local/bin/

.PHONY: all clean install