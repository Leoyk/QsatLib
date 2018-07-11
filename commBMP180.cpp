
//static for PT


#include "commBMP180.h"
#include "Arduino.h"


#define BMP180ADD 0x77  // I2C address of BMP180  
                       
  // Oversampling Setting           0: single    1: 2 times    2: 4 times   3: 8 times 
#define OSS 3      
//修改此处需要修改延时时间
  
  
  
/**********************MSB      LSB******/
static int ac1;           // 0xAA     0xAB
static int ac2;           // 0xAC     0xAD
static int ac3;           // 0xAE     0xAE
static unsigned int ac4;  // 0xB0     0xB1
static unsigned int ac5;  // 0xB2     0xB3
static unsigned int ac6;  // 0xB4     0xB5
static int b1;            // 0xB6     0xB7
static int b2;            // 0xB8     0xB9
static int mb;            // 0xBA     0xBB
static int mc;            // 0xBC     0xBD
static int md;            // 0xBE     0xBF
static long b5;          

int cnt = 0;


void pressureInit(){
  ac1 = bmp180ReadDate(0xAA);                      //get full data
  ac2 = bmp180ReadDate(0xAC);  
  ac3 = bmp180ReadDate(0xAE);  
  ac4 = bmp180ReadDate(0xB0);  
  ac5 = bmp180ReadDate(0xB2);  
  ac6 = bmp180ReadDate(0xB4);  
  b1  = bmp180ReadDate(0xB6);  
  b2  = bmp180ReadDate(0xB8);  
  mb  = bmp180ReadDate(0xBA);  
  mc  = bmp180ReadDate(0xBC);  
  md  = bmp180ReadDate(0xBE);
  
//get baseline  
  
}
/*
  temperature = bmp180GetTemperature(bmp180ReadUT());
  temperature = temperature*0.1;
  pressure = bmp180GetPressure(bmp180ReadUP());
  pressure2 = pressure/101325;
  pressure2 = pow(pressure2,0.19029496);
  altitude = 44330*(1-pressure2);                            //altitude = 44330*(1-(pressure/101325)^0.19029496);
*/


/***BMP180 temperature Calculate***/
float tempVal()
{
  static long x1, x2;
  unsigned int ut = getTV();
  
  x1 = 0;
  x2 = 0;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;  //x1=((ut-ac6)*ac5)/(2^15)
  x2 = ((long)mc << 11)/(x1 + md);                //x2=(mc*2^11)/(x1+md)
  b5 = x1 + x2;                                   //b5=x1+x2
  return (((b5 + 8)>>4)*0.1);                           //t=(b5+8)/(2^4)
}

/***BMP180 pressure Calculate***/

double pressureVal()
{
  static long x1, x2, x3, b3, b6, p;
  static unsigned long b4, b7;
  
  unsigned long up = getPV();
  
  x1= x2= x3= b3= b6= p=0;
  b4= b7= 0;  
  
  
  
  b6 = b5 - 4000;

  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}



// Altitude =(44330.0 * (1.0-pow((float)(pressure) / 101325, 1.0/5.255)) ); 
double altitudeVal(double pre,long baseline){
  
  double pre2;
  pre2 = pre/baseline;
  pre2 = pow(pre2,0.19029496);
  
  return 44330*(1-pre2); 
}



/*** Read 1 bytes from the BMP180  ***/

int bmp180Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP180ADD);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP180ADD, 1);
  
  cnt = 500;
  while(!Wire.available() && cnt --){
	  delay(1);
  }
    
  return Wire.read();
}

/*** Read 2 bytes from the BMP180 ***/
int bmp180ReadDate(unsigned char address)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP180ADD);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP180ADD, 2);
  
  cnt = 500;
  while((Wire.available()<2) && cnt --){
	  delay(1);
  }
  
  msb = Wire.read();
  lsb = Wire.read();
  return (int) msb<<8 | lsb;
}







void orderTV()
{
  Wire.beginTransmission(BMP180ADD);
  Wire.write(0xF4);                       // Write 0x2E into Register 0xF4
  Wire.write(0x2E);                       // This requests a temperature reading
  Wire.endTransmission();  
  }

// delay(5);                               // Wait at least 4.5ms
  
/*** read uncompensated temperature value ***/
unsigned int getTV()
{
  unsigned int ut;

  ut = bmp180ReadDate(0xF6);               // read MSB from 0xF6 read LSB from (16 bit)
  return ut;
}





void orderPV(){//delay 8,dependent on oss
  Wire.beginTransmission(BMP180ADD);
  Wire.write(0xF4);                        // Write 0x34+(OSS<<6) into register 0xF4
  Wire.write(0x34 + (OSS<<6));             // 0x34+oss*64
  Wire.endTransmission(); 
}

  
/*** Read uncompensated pressure value from BMP180 ***/
unsigned long getPV()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  

  Wire.beginTransmission(BMP180ADD);
  Wire.write(0xF6);                        // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.endTransmission();
  
  Wire.requestFrom(BMP180ADD, 3);


  cnt = 500;
  while((Wire.available() < 3)&&(cnt --)){
	delay(1); 
  }           // Wait for data to become available
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);//16 to 19 bit
  return up;
}

