/*
 * GPS.h
 *
 *  Created on: Feb 13, 2019
 *      Author: Shantanu
 */

#ifndef GPS_H_
#define GPS_H_

#define DEGTORAD     0.0174532925199432957f
#define RADTODEG     57.295779513082320876f
#define EARTH_RADIUS 6371000


struct gpgga_struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
    double latitude;
    char latdir;
    double longitude;
    char londir;
    uint8_t fixquality;
};

struct gprmc_struct {
    double latitude;
    char latdir;
    double longitude;
    char londir;
    double speed;
    double course;
    uint8_t status;
};

struct gps_struct {
    double lat;
    char latdir;
    double lon;
    char londir;
    double speed;
    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
};

void gpsInit(void);
void initStruct(struct gps_struct *gp);
void initGGAStruct(struct gpgga_struct* gp);
void initRMCStruct(struct gprmc_struct* gp);
struct gpgga_struct  gpsReadGGA(void);
uint8_t gpsParseGGA(void);
uint8_t gpsParseRMC(void);
struct gps_struct gpsReadNMEA(void);
struct gps_struct gpsReadNMEA2(void);
void gpsConvertData(double *latitude, char ns,  double *longitude, char we);
double gpsCovertToDec(double deg_point);
double gpsGetDistance(struct gps_struct* a, struct gps_struct* b);
double gpsGetBearing(struct gpgga_struct* a, struct gpgga_struct* b);
uint8_t gpsGetSeconds(struct gps_struct* prev, struct gps_struct* curr);

#endif /* GPS_H_ */
