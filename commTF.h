#ifndef __COMMTF_H__
#define __COMMTF_H__

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

void tfInit();
void preFile(String* name);
void writeTF(String a);
void wirteLn();



#endif