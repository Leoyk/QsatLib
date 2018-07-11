#ifndef _COMMBMP180_H__
#define _COMMBMP180_H__
#include"Wire.h"

// Oversampling Setting           0: single    1: 2 times    2: 4 times   3: 8 times 


void pressureInit();

void orderPV();//延时30ms之后才能获取压力值
double pressureVal();//获取到的气压值 单位：Pa


double altitudeVal(double pre,long baseline);//传入气压换算海拔高度值 单位：m ,基准面高度：baseline 101325


void orderTV();//延时5ms之后才能获取温度值
float tempVal();//获取到的温度值 单位：℃



unsigned long getPV();
unsigned int getTV();


int bmp180Read(unsigned char address);
int bmp180ReadDate(unsigned char address);


#endif