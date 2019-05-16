/*
 * GPS.c
 *
 *  Created on: Feb 13, 2019
 *      Author: Shantanu
 */
#include "../HWI/UART.h"
#include "msp430.h"
#include "string.h"
#include "GPS.h"
#include "stdlib.h"
#include "math.h"

#define STRLEN 7
#define BUFMAX 100
char dataBuf[BUFMAX];
char searchGGA[STRLEN] = "$GPGGA";
char searchRMC[STRLEN] = "$GPRMC";
struct gpgga_struct gpgga;
struct gprmc_struct gprmc;


//Testing
float timef;

void gpsInit(void) {
    // Initialize UART at 9600 baud rate.
    uartInit();
    initGGAStruct(&gpgga);
    initRMCStruct(&gprmc);
}

void initStruct(struct gps_struct *gp) {
    gp->lat = 0.0;
    gp->latdir = 0;
    gp->lon = 0.0;
    gp->londir = 0;
    gp->speed = 0.0;
    gp->hour = 0;
    gp->minute = 0;
    gp->seconds = 0;
}

void initGGAStruct(struct gpgga_struct* gp) {
    gp->latitude = 0.0;
    gp->latdir = 0;
    gp->longitude = 0.0;
    gp->londir = 0;
    gp->fixquality = 0;
    gp->hour = 0;
    gp->minute = 0;
    gp->seconds = 0;
}

void initRMCStruct(struct gprmc_struct* gp) {
    gp->latitude = 0.0;
    gp->latdir = 0;
    gp->longitude = 0.0;
    gp->londir = 0;
    gp->speed = 0;
    gp->course = 0;
}

void gpsConvertData(double *latitude, char ns,  double *longitude, char we) {
    double lat = (ns == 'N') ? *latitude : -1 * (*latitude);
    double lon = (we == 'E') ? *longitude : -1 * (*longitude);

    *latitude = gpsCovertToDec(lat);
    *longitude = gpsCovertToDec(lon);
}

double gpsCovertToDec(double deg_point) {
    double ddeg;
    double sec = modf(deg_point, &ddeg)*60;
    int deg = (int)(ddeg/100);
    int min = (int)(deg_point-(deg*100));

    double absdlat = round(deg * 1000000.);
    double absmlat = round(min * 1000000.);
    double absslat = round(sec * 1000000.);

    return round(absdlat + (absmlat/60) + (absslat/3600)) /1000000;
}

uint8_t gpsParseGGA(void) {
    // Create another point so we can manipulate it.
    char *p = &dataBuf[0];
    // Do we need time? Skip for now.
    p = strchr(p, ',')+1;
    timef = atof(p);
    uint32_t time = timef;
    gpgga.hour = time / 10000;
    gpgga.minute = (time % 10000) / 100;
    gpgga.seconds = (time % 100);

    // Get latitude.
    p = strchr(p, ',')+1;
    gpgga.latitude = atof(p);
    // Get latitude direction. Use switch case to handle no value. i.e. getting a ','.
    p = strchr(p, ',')+1;
    switch (p[0]) {
    case 'N':
        gpgga.latdir = 'N';
        break;
    case 'S':
        gpgga.latdir = 'S';
        break;
    case ',':
        gpgga.latdir = '\0';
        break;
    }
    // Now, longitude.
    p = strchr(p, ',')+1;
    gpgga.longitude = atof(p);
    // Same logic as latitude to get longitude direction.
    p = strchr(p, ',')+1;
    switch (p[0]) {
    case 'W':
        gpgga.londir = 'W';
        break;
    case 'E':
        gpgga.londir = 'E';
        break;
    case ',':
        gpgga.londir = '\0';
        break;
    }
    // Get fix quality. It's 1 when we have a fix.
    p = strchr(p, ',')+1;
    gpgga.fixquality = (uint8_t)atoi(p);
    return gpgga.fixquality;
}


uint8_t gpsParseRMC(void)
{
    char *p = &dataBuf[0];

    p = strchr(p, ',')+1; //skip time

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'A':
            gprmc.status = 1;
            break;
        case 'S':
            gprmc.status = 0;
            break;
        case ',':
            gprmc.status = 0;
            break;
    }

    p = strchr(p, ',')+1;
    gprmc.latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            gprmc.latdir = 'N';
            break;
        case 'S':
            gprmc.latdir = 'S';
            break;
        case ',':
            gprmc.latdir = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    gprmc.longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            gprmc.londir = 'W';
            break;
        case 'E':
            gprmc.londir = 'E';
            break;
        case ',':
            gprmc.londir = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    gprmc.speed = atof(p);

    p = strchr(p, ',')+1;
    gprmc.course = atof(p);

    return gprmc.status;
}

struct gps_struct gpsReadNMEA(void) {
    uint8_t byteRecv = 0x00;
    struct gps_struct data;
    // Clear the buffer and set it to 0.
    uint8_t ggadoneflag = 0, rmcdoneflag = 0;
    unsigned int bufcount = 0;
    memset(&dataBuf[0], 0, sizeof(dataBuf));

    while(!(ggadoneflag && rmcdoneflag)) {
        if (UCA0IFG & UCRXIFG) {
            byteRecv = uartReceiveChar();
            dataBuf[bufcount] = byteRecv;
            UCA0IFG &= ~UCRXIFG;
            if (dataBuf[0] != '$') {
                bufcount = 0;
                memset(&dataBuf[0], 0, sizeof(dataBuf));
            }
            else {
                bufcount++;
                if (byteRecv == '\n') {
                    if (strncmp(dataBuf, searchGGA, 6) == 0) {
                        initGGAStruct(&gpgga);
                        ggadoneflag = gpsParseGGA();
                    }
                    else if (strncmp(dataBuf, searchRMC, 6) == 0) {
                        initRMCStruct(&gprmc);
                        rmcdoneflag = gpsParseRMC();
                    }
                    bufcount = 0;
                    memset(&dataBuf[0], 0, sizeof(dataBuf));
                    if (!ggadoneflag) {
                        initGGAStruct(&gpgga);
                    }
                    if (!rmcdoneflag) {
                        initRMCStruct(&gprmc);
                    }
                }
            }
        }
    }
    data.lat = gpgga.latitude;
    data.latdir = gpgga.latdir;
    data.lon = gpgga.longitude;
    data.londir = gpgga.londir;
    data.hour = gpgga.hour;
    data.minute = gpgga.minute;
    data.seconds = gpgga.seconds;
    data.speed = gprmc.speed;

    return data;
}

struct gps_struct gpsReadNMEA2(void) {
    uint8_t byteRecv = 0x00;
    struct gps_struct data;
    // Clear the buffer and set it to 0.
    uint8_t ggadoneflag = 0;
    unsigned int bufcount = 0;
    memset(&dataBuf[0], 0, sizeof(dataBuf));

    while(!ggadoneflag) {
        if (UCA0IFG & UCRXIFG) {
            byteRecv = uartReceiveChar();
            dataBuf[bufcount] = byteRecv;
            UCA0IFG &= ~UCRXIFG;
            if (dataBuf[0] != '$') {
                bufcount = 0;
                memset(&dataBuf[0], 0, sizeof(dataBuf));
            }
            else {
                bufcount++;
                if (byteRecv == '\n') {
                    if (strncmp(dataBuf, searchGGA, 6) == 0) {
                        initGGAStruct(&gpgga);
                        ggadoneflag = gpsParseGGA();
                    }
                    bufcount = 0;
                    memset(&dataBuf[0], 0, sizeof(dataBuf));
                    if (!ggadoneflag) {
                        initGGAStruct(&gpgga);
                    }                }
            }
        }
    }
    data.lat = gpgga.latitude;
    data.latdir = gpgga.latdir;
    data.lon = gpgga.longitude;
    data.londir = gpgga.londir;
    data.hour = gpgga.hour;
    data.minute = gpgga.minute;
    data.seconds = gpgga.seconds;
    data.speed = 0.0;

    return data;
}


double gpsGetDistance(struct gps_struct* a, struct gps_struct* b) {
    // Get the delta values for latitude and longitude.
    double dlat = (b->lat - a->lat) * DEGTORAD;
    double dlong = (b->lon - a->lon) * DEGTORAD;

    // Convert the latitudes to radian.
    double lat1 = a->lat * DEGTORAD;
    double lat2 = b->lat * DEGTORAD;

    // Apply the haversine formula.
    double x = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlong/2) * sin(dlong/2);
    double y = 2 * atan2(sqrt(x), sqrt(1-x));

    // Convert radian distance by multiplying it with earth's radius.
    return EARTH_RADIUS * y;
}

double gpsGetBearing(struct gpgga_struct* a, struct gpgga_struct* b) {
    // Get the delta values for latitude and longitude.
    double dlong = (b->longitude - a->longitude) * DEGTORAD;

    // Convert the latitudes to radian.
    double lat1 = a->latitude * DEGTORAD;
    double lat2 = b->latitude * DEGTORAD;

    double y = sin(dlong) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dlong);
    // This formula is for the initial bearing (sometimes referred to as forward azimuth). i.e. bearing angle from point a to point b.
    return atan2(y, x) * RADTODEG;
}

uint8_t gpsGetSeconds(struct gps_struct* prev, struct gps_struct* curr) {
    int seconds1 = prev->hour*60*60 + prev->minute*60 + prev->seconds;
    int seconds2 = curr->hour*60*60 + curr->minute*60 + curr->seconds;
    return (seconds2-seconds1);
}
