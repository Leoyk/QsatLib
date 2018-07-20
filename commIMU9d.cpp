#include <Wire.h>
#include "commIMU9d.h"
#include "Arduino.h"

////magnetometer
#define address 0x1E //001 1110b(0x3C>>1), HMC5883的7位i2c地址  
#define MagnetcDeclination 6.52 //北京地磁偏角
#define CalThreshold 0  
int offsetX,offsetY,offsetZ; 

////mpu
#define    MPU6050_ADDRESS            0x68

#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define fliterNumber 20


double smoothFliterAX(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	
	double smoothFliterAY(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	
	double smoothFliterAZ(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	double smoothFliterGX(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	
	double smoothFliterGY(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	double smoothFliterGZ(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	double smoothFliterMX(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	double smoothFliterMY(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }
	double smoothFliterMZ(double newRcVal){//参数为元素个数
//创建滑动窗口数组并启用更新

  static double arr[fliterNumber];//窗口数组
  static double sum;//N个数值的总和
  double outVal;//滤波后的输出值
 
  sum = sum - arr[fliterNumber - 1];//去除最后一位的总和
  
  for(int i = fliterNumber - 1;i > 0;i --){
        arr[i]=arr[i - 1];//从最后一位开始向右滑动一次，最后一位被丢弃；
  }
  
  arr[0] = newRcVal;//首位存入新的数据
  
  sum = sum + arr[0];//新的总和
  
  outVal = sum / fliterNumber;//求算数平均值

 return outVal;
    }

	
volatile void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

volatile void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

volatile void imu9dInit(){
  // Set accelerometers low pass filter at 1000Hz
  I2CwriteByte(MPU6050_ADDRESS,0x19,7);
  // Set gyroscope low pass filter at 1000Hz
  I2CwriteByte(MPU6050_ADDRESS,0x1A,0x00);
  
  // Configure gyroscope range
  I2CwriteByte(MPU6050_ADDRESS,0x1B,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU6050_ADDRESS,0x1C,ACC_FULL_SCALE_2_G);
	
  // PLL with X axis gyroscope reference and disable sleep mode
  I2CwriteByte(MPU6050_ADDRESS,0X6B,01);
  
  //设置HMC5883模式   选择配置寄存器A   0111 0000b，具体配置见数据手册 	
  I2CwriteByte(address,0x00,0x70);	
  
  I2CwriteByte(address,0x02,0x00);	//选择模式寄存器  //连续测量模式:0x00,单一测量模式:0x01 
		
   
 // calibrateMag();  //旋转校正磁场	

  
}

// Correct angle
float correctAngle(float heading) {
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }
  return heading;
}





  uint8_t Buf[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  uint8_t Mag[7]={0,0,0,0,0,0,0}; 
 
  
volatile bool commIMU9d::getData(){

  for(int i = 0;i < 14;i ++){
	Buf[i] = 0;
  }
  I2Cread(MPU6050_ADDRESS,0x3B,14,Buf);	
  
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];
  
  // Accelerometer +- 8g
  AX = ax + 116; 
  AY = ay - 41;
  AZ = az + 39;  
  
  // AX = smoothFliterAX(AX);
  // AY = smoothFliterAY(AY);
  // AZ = smoothFliterAZ(AZ); 
  
  
  // Gyroscope +- 1000
  GX = gx - 81; 
  GY = gy - 22;
  GZ = gz + 31;  
  
  // GX = smoothFliterGX(GX);
  // GY = smoothFliterGY(GY);
  // GZ = smoothFliterGZ(GZ); 
  // Read register Status 1 and wait for the DRDY: Data Ready
 
 int mx,my,mz;
  getRawData(&mx,&my,&mz);  
  
  // Magnetometer  2940 / 810 uT
  MX = mx*0.3955; 
  MY = my*0.3955;
  MZ = mz*0.3955;  
  
   MX = smoothFliterMX(MX);
   MY = smoothFliterMY(MY);
   MZ = smoothFliterMZ(MZ);
   HEAD = calculateHeading(&mx,&my,&mz);
}


   
void getRawData(int* x ,int* y,int* z)  
{  
  Wire.beginTransmission(address);  
  Wire.write(0x03); //从寄存器3开始读数据  
  Wire.endTransmission();  
  //每轴的数据都是16位的  
  Wire.requestFrom(address, 6);  
  if(6<=Wire.available()){  
    *x = Wire.read()<<8; //X msb，X轴高8位  
    *x |= Wire.read(); //X lsb，X轴低8位  
    *z = Wire.read()<<8; //Z msb  
    *z |= Wire.read(); //Z lsb  
    *y = Wire.read()<<8; //Y msb  
    *y |= Wire.read(); //Y lsb  
  }  
}  
   
float calculateHeading(int* x ,int* y,int* z)  
{  
  float headingRadians = atan2((double)((*y)),(double)((*x)));  
  // 保证数据在0-2*PI之间  
  if(headingRadians < 0)  
    headingRadians += 2*PI;  
   
  int headingDegrees = headingRadians * 180/M_PI;  
  headingDegrees += MagnetcDeclination; //磁偏角  
   
  // <span style="font-family: Arial, Helvetica, sans-serif;">保证数据在0-360之间</span>  
  if(headingDegrees > 360)  
    headingDegrees -= 360;  
   
  return headingDegrees;  
}  
   
void calibrateMag()  
{  
  int x,y,z; //三轴数据  
  int xMax, xMin, yMax, yMin, zMax, zMin;  
  //初始化  
  getRawData(&x,&y,&z);    
  xMax=xMin=x;  
  yMax=yMin=y;  
  zMax=zMin=z;  
  offsetX = offsetY = offsetZ = 0;  
   
  Serial3.println("Starting Calibration......");  
  Serial3.println("Please turn your device around in 20 seconds");  
   
  for(int i=0;i<2000;i++)  
  {  
    getRawData(&x,&y,&z);  
    // 计算最大值与最小值  
    // 计算传感器绕X,Y,Z轴旋转时的磁场强度最大值和最小值  
    if (x > xMax)  
      xMax = x;  
    if (x < xMin )  
      xMin = x;  
    if(y > yMax )  
      yMax = y;  
    if(y < yMin )  
      yMin = y;  
    if(z > zMax )  
      zMax = z;  
    if(z < zMin )  
      zMin = z;  
   
    delay(10);  
   
    if(i%10 == 0)  
    {  
      Serial.print(xMax);  
      Serial.print(" ");  
      Serial.println(xMin);  
    }  
  }  
  //计算修正量  
  if(abs(xMax - xMin) > CalThreshold )  
    offsetX = (xMax + xMin)/2;  
  if(abs(yMax - yMin) > CalThreshold )  
    offsetY = (yMax + yMin)/2;  
  if(abs(zMax - zMin) > CalThreshold )  
    offsetZ = (zMax +zMin)/2;  
   
  Serial3.print("offsetX:");  
  Serial3.print("");  
  Serial3.print(offsetX);  
  Serial3.print(" offsetY:");  
  Serial3.print("");  
  Serial3.print(offsetY);  
  Serial3.print(" offsetZ:");  
  Serial3.print("");  
  Serial3.println(offsetZ);  
}  







