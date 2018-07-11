#ifndef _COMMIMU_H__
#define _COMMIMU_H__

#include <Wire.h>



volatile void imuInit();


class commIMU{
	public:
		double  AX;
		double  AY;
		double  AZ;
		double  GX;
		double  GY;
		double  GZ;
		double  MX;
		double  MY;
		double  MZ;
		volatile bool getData();
	private:
	
};


#endif
