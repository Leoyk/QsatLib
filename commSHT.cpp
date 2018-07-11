#include "commSHT.h"
#include <Arduino.h>
#include <Wire.h>
#include "THbase.h"
///////////////////////////////////////////////////温湿度传感器数据
Adafruit_SHT31 sht31 = Adafruit_SHT31();


/*================================================================ 
* 函 数 名：
* shtInit 
* 
* 参 数：
* 传感器地址
*
* 功 能 描 述: 
* 需要开启Wire.begin();然后此函数初始化传感器。
* 
* 返 回 值：
* 获取的紫外线传感器电压值（mv）
* 
* 作 者：刘要坤 2018年5月24日14:31:48

================================================================*/ 
void shtInit(int add){
  //温湿度传感器设置 
  sht31.begin(add);//0x44
}




void orderTH(){
	sht31.orderTh();
}


void getTH(float *t,float *h){
	sht31.getTh(t,h);
}
 
