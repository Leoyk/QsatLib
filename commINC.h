
//IIC
#include "Wire.h"
/*
 在初始化函数中调用
 Wire.begin();
*/


//SerialServo
 #include "commCAM.h"
/*
摄像头接硬件串口3


//初始化摄像头： 检测SD卡 
函数1：void camInit(); 
用法：初始化函数中调用，调用结束后需要等待200ms然后清空摄像头缓存
参数：无参数
返回值：无	
  
  
//清空缓存 
函数2：void clearInput();
用法：清空摄像头串口的接收缓存
参数：无参数
返回值：无	
  
 //复位摄像头 不返回任何值，用后可能需要清空串口缓存 
函数3：void orderReset();
用法：直接调用。
参数：无参数
返回值：无	


//请求版本号 调用后需延时> 10ms才能获取版本号
函数4：void orderVersion();
用法：直接调用
参数：无参数
返回值：无	


//获得版本号 开启串口输出情况下会打印出版本号   返回
函数5：bool getVersion();
用法：order之后等待10ms之后读取
参数：无参数
返回值：1：无错误，返回0：错误；


  

//1 stop pic 
	//请求停止刷新图像 调用后需延时> 10ms才能获取反馈
	void orderStop();
	
	//获取停止状态： 0：操作成功，1：操作失败；
	bool getStop();

//2 get lenth 
	//请求获取图像数据长度 调用后需延时> 10ms才能获取反馈
	void orderLenth();
	
	//将获取的长度返回到*lenth中  1：操作成功，0：操作失败；
	bool getLenth(long *lenth);

//3 get pic
	//创建文件并打开
	void preFile(String* name);
	
	//请求获取图像数据 调用后需延时> 10ms才能获取反馈
	bool orderPic();
	
	void getPic();
	
	//关闭文件
	void closeFile();

//4 refresh img 初始化变量并刷新图像
	void refreshImg();
	
	//初始化变量
	void initV();


*/




//SerialServo
 #include "commServo.h"
/*



设置软串口，名为SSerial RX TX
SoftwareSerial  SSerial(50, 51);
  
  
  
函数1：void ssInit(void );
用法：初始化函数中调用,初始化软串口波特率
参数：无参数
返回值：无	
  
  
函数2：void changeID(uint8_t oldID, uint8_t newID);
用法：修改地址。知道原地址可以修改新地址，原地址设置为254时面向所有连接的对象广播
参数：oldID：原地址；  newID： 新地址。
返回值：无	
  
  
函数3：void moveToAngle(int ID,int angle,int time);//此函数会使电机上力 
用法：指定ID的舵机，运动到指定的角度angle，运动时间为time，此函数需要添加额外的延时。
参数：ID ：0~253； angle：角度：0~240dge；time：0~30000ms
返回值：无	

  
函数4：unLoad(int ID);
用法：指定ID的舵机脱力。
参数：ID ：0~253；
返回值：无	


  
函数5：load(int ID);
用法：指定ID的舵机上力。
参数：ID ：0~253；
返回值：无	


  
函数6：double readAngle(int ID);
用法：指定ID的舵机现在的角度。
参数：ID ：0~253；
返回值：角度：0~240dge



*/



//WS2812
 #include "commWS2812.h"
/*


  #define PINRIGHT  2
  #define PINLEFT   3  
  #define PINMIDDLE 4
  
  
  
函数1：void wsInit(void );
用法：初始化函数中调用,初始化三个灯阵 
参数：无参数
返回值：无	


函数2：
void setBrightness(String lmr,int bright);
用法：直接调用，设置亮度值 默认为40
参数：lmr（"left"、"right"、"middle"）
返回值：无	

函数3：
void clearPixels(String lmr);
用法：清空点阵
参数：lmr（"left"、"right"、"middle"）
返回值：无	


函数4：void setPixels(String lmr,int hang,int hex,int red,int green,int blue);
用法：直接调用，显示点，在lmr灯板的hang行点亮lie二进制代表的灯，颜色为rgb
参数：灯板号：lmr（"left"、"right"、"middle"）行：hang（0~7），列：lie（0b00000000），rgb分别对应红绿蓝（0~255）
返回值：无	

函数5：void showPixels(String lmr);
用法：开启lmr点阵显示
参数：lmr（"left"、"right"、"middle"）
返回值：无	

函数6：bool showStrLeft(String a,int red,int green,int blue);
用法：直接调用，在left灯板上显示字符串a；
	while(showStrLeft("AAA",0,0,200) == 0){
		delay(80);
	}
参数：a：要显示的字符串，rgb分别对应红绿蓝（0~255）
返回值：无	

函数7：bool showStrRight(String a,int red,int green,int blue);
用法：直接调用，在right灯板上显示字符串a；
	while(showStrRight("AAA",0,0,200) == 0){
		delay(80);
	}
参数：a：要显示的字符串，rgb分别对应红绿蓝（0~255）
返回值：无	


*/



//姿态                     T
#include "commIMU.h"
/*

//      m/s^2
//      deg/s
//       uT


函数1：imuInit();
用法：初始化函数中调用，要求先开启 Wire.begin();
参数：无参数
返回值：无


class commIMU{
	public:
		float AX;
		float AY;
		float AZ;
		float GX;
		float GY;
		float GZ;
		float MX;
		float MY;
		float MZ;
		void getData();
	private:
	
};

函数2：IMU.getData();
用法：定义IMU然后初始化，再调用getData;
参数：无参数
返回值：成功1 失败0

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

//UV
#include "commUV.h"
/*
函数1：s12sdValMv(int s12sdPin);
用法：直接调用
参数：模拟量读取口
返回值：返回浮点型电压值

函数2：uvLevel(int s12sdPin);
用法：直接调用
参数：模拟量读取口
返回值：返回整型紫外线指数

*/

//电压检测
#include "commCDC.h"
/*

函数1：dcVal(int voltagePin);
用法：直接调用
参数：模拟量读取口
返回值：返回浮点型电压值
*/

//光照强度检测              T
#include "commLight.h"
/*
函数1：TSL2561Init();
用法：初始化函数中调用，要求先开启 Wire.begin();
参数：无参数
返回值：无

函数2：lightVal();
用法：直接调用 
参数：无参数
返回值：浮点型光照流明值
*/

//UHF                  
#include "commUHF.h"
/*
函数1：void FMinit(long Boud);
用法：初始化函数中调用，要求设置串口3的波特率;
参数：串口3的波特率
返回值：无


函数2：void enAT(int pin);
用法：直接调用
参数：进入AT模式，pin连接set
返回值：无返回值

函数3：outAT(int pin);
用法：直接调用
参数：退出AT模式
返回值：无返回值

***测试用函数4：void testAt();
用法：在开启AT模式之后调用，且与必须在上一条指令50ms之后运行
参数：无
返回值：无返回值

函数5：void setBoud(long a);
用法：设置UHF波特率，在开启AT模式之后调用，且与必须在上一条指令50ms之后运行
参数：波特率数值 u long型
返回值：无返回值

函数6：void setCh(long a);
用法：设置UHF信道，在开启AT模式之后调用，且与必须在上一条指令50ms之后运行
参数：信道数值 int型
返回值：无返回值

*/



//FM                  
#include "commFM.h"
/*
函数1：FMinit();
用法：初始化函数中调用，要求设置软串口的对应引脚，在.c文件中;
参数：无参数
返回值：无


函数2：void setFre(int a);
用法：直接调用
参数：设置的频率，范围885~1080，无小数点
返回值：无返回值

函数3：void setVol(int a);
用法：直接调用
参数：设置的音量，范围0~30，无小数点
返回值：无返回值

函数4：void orderState();
用法：请求状态值，应用于获取状态值之前，请求后至少80ms后才能调用获取函数
参数：无参数
返回值：无返回值


函数5：bool getState(int *volume,int *frequncy);
用法：//获取状态值，成功返回1；失败，返回0；分别传到两个变量里，frequency为int型，当调整模块的音量或频率时获取的数据可能会失败
参数：音量、频率
返回值：无返回值

int getEcho()

返回获取到的数字数据，可以用来校验设置是否成功，需要在设置后10ms之后调用

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




