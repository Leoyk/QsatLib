
//IIC
#include "Wire.h"
/*
 在初始化函数中调用
 Wire.begin();
*/


//5883
//MPU6050
#include "commIMU9d.h"
/*

*/


#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <SimpleKalmanFilter.h>

//TF
#include "commTF.h"
/*

*/


#include"Servo.h"

//GPS
#include "commGPS.h"
/*
void GPS_init();
void location();

bool GPS_Parsing(void);
double Calculate_GPS_Distance(double lat1, double lng1, double lat2, double lng2);
*/



//温湿度sht31
#include "commSHT.h"
/*

函数1：shtInit(int add);
用法：初始化函数中调用，要求先开启 Wire.begin();
参数：传感器地址，SHT31默认地址0x44;
返回值：无


//在调用获取温湿度之前必须先执行此步骤，然后等待500ms
void orderTH();

//温度 ,湿度
void getTH(float *t,float *h);

*/

//电压检测
#include "commCDC.h"
/*

函数1：dcVal(int voltagePin);
用法：直接调用
参数：模拟量读取口
返回值：返回浮点型电压值
*/

//RTC                  
#include "commRTC.h"
/*
函数1：RTCinit();
用法：初始化函数中调用，要求先开启 Wire.begin();
参数：无参数
返回值：无

函数2：adjTime(int y,int m,int d,int h,int mi,int s);
用法：直接调用
参数：年月日时分秒
返回值：无

函数3：getTime(int * year,int * mon ,int * day ,int * hour ,int * min ,int * sec);
用法：直接调用
参数：年月日时分秒
返回值：无

*/

//气压、温度、高度检测                  T
#include "commBMP180.h"
/*
函数1：pressureInit();初始化气压计
用法：初始化函数中调用，要求先开启 Wire.begin();
参数：无参数
返回值：无




函数2：orderPV();请求气压值
用法：直接调用 
参数：无参数
返回值：无返回值;

函数3：pressureVal();获取气压值
用法：先调用orderPV()，30ms后才能获取数据
参数：无参数
返回值：浮点型气压值 单位pa;



函数4：double altitudeVal(double pre,long baseline)
用法：获取完气压值之后才能调用
参数：气压值 获取高度 ,基准面高度：baseline 101325
返回值：浮点型海拔高度 单位：m;



函数5：orderTV();//请求温度值
用法：直接调用
参数：无参数
返回值：无返回值

函数6：tempVal();//获取温度值
用法：先调用orderTV，5ms之后才能获取数据
参数：无参数
返回值：浮点型温度值 ℃;

*/




