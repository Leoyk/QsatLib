#ifndef _COMMGPS_H
#define _COMMGPS_H

#include <Adafruit_GPS.h>

struct commGD{
		float  latitude;
		float  longitude;
		float  altitude;
		uint8_t fix;
		uint8_t fixq;
		
		
		float speed;
		float  HDOP = 100;
		uint8_t sat = 0;
		
		uint8_t minu = 0;
		uint8_t hour = 0;
		uint8_t sece = 0;
		
};

void GPS_init();
void location(struct commGD *commGPS);
void GPS_Parsing(struct commGD *commGPS);
double Calculate_GPS_Distance(double lat1, double lng1, double lat2, double lng2);


bool getLocal(struct commGD *commGPS);

#endif
