#include "commCDC.h"
#include "Arduino.h"



/*================================================================ 
* 函 数 名：
* dcVal() 
* 
* 参 数：
* 读取模拟量的引脚
*
* 功 能 描 述: 
* 检测输入电压
* 
* 返 回 值：
* 浮点型 0~25v电压
* 
* 作 者：赫文 2018年5月18日
================================================================*/ 
float dcVal(int voltagePin){
   float  Value=0.00;
   Value=analogRead(voltagePin) * 0.02445;  
   return Value;
}