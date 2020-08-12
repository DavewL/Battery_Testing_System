#include "LED.h"
#include "application.h"

//int led1 = D7;

void initLED(void){
  //pinMode(led1, OUTPUT);
}

void toggleLED(void){
  static int prevState = 0;

  if (prevState == 0){
    //digitalWrite(led1, HIGH);
    prevState = 1;
  }
  else{
    //digitalWrite(led1, LOW);
    prevState = 0;
  }
}

void LEDon(void){
  //digitalWrite(led1, HIGH);
}

void LEDoff(void){
  //digitalWrite(led1, LOW);
}
