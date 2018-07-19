#include"commTF.h"


Sd2Card card;

File dataFile;

char filename[10];

#define chipSelect 5 

void tfInit(){

 if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }   
  
}


void preFile(String* name){
  
  strcpy(filename, "log00.txt");
  
  for (int i = 0; i < 100; i++) {
    filename[3] = '0' + i/10;
    filename[4] = '0' + i%10;
    
    if (! SD.exists(filename)) {
        break;
      }
  }

  dataFile = SD.open(filename, FILE_WRITE);  
  *name = filename;
}

void writeTF(String a){
	dataFile.print(a);
	dataFile.close();
}  

void wirteLn(){
	dataFile.println();
	dataFile.close();
}  
