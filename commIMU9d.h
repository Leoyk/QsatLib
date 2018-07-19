#ifndef _COMMIMU9D_H__
#define _COMMIMU9D_H__



volatile void imu9dInit();


class commIMU9d{
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
		double  HEAD;
		volatile bool getData();
	private:

	
};
	void getRawData(int* x ,int* y,int* z) ;
	float calculateHeading(int* x ,int* y,int* z);
	void calibrateMag() ;
float correctAngle(float heading);
#endif
