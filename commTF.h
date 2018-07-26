#ifndef __COMMTF_H__
#define __COMMTF_H__

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

void tfInit();
void preFile();

void openFile();
void iwriteData(int a);
void iwriteDataLn(int a);

void writeData(float a);
void writeDataLn(float a);
void SwriteData(String a);
void SwriteDataLn(String a);
void closeFile();

#endif




/**************************
data

0:airT	1:airH	

2:baseP	3:pres	4:alti

5:distance	6:latitude	7:longitude	8:altitude	9:sat	10:HDOP

11:fix	12:fixq	13:gSpe	14:bLa	15:bLo	16:hour	17:min	18:sec	

19:AX	20:AY	21:AZ	22:GX	23:GY	24:GZ	25:MX	26:MY	27:MZ	28:PITCH	29:ROLL	30:YAW	31:G



**************************/