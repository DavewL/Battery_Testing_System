//Cummins BMC Message Protocol
#include "CumminsCAN.h"
#include "defines.h"
#include "Globals.h"
#include "TickTimer.h"

static const unsigned int scawCumminsTimers[NUM_CUMMINS_TIMERS] =
{
  /* CT_CUMMINS_LOST_DELAY   */   1000,  //ms -delay before CUMMINS CAN is assumed lost
};

static TICK_TIMER_S scastCumminsTimers[NUM_CUMMINS_TIMERS];

void ResetCumminsTimer(CUMMINS_TIMERS eCANTimer);


void initCumminsCAN(void){
  unsigned int wIndex;
  for (wIndex = 0; wIndex < ((unsigned int)NUM_CUMMINS_TIMERS); wIndex++){
   RegisterTimer(&scastCumminsTimers[wIndex]);
   ResetCumminsTimer((CUMMINS_TIMERS)wIndex);
  }
  battCell1mv = 0;
  battCell2mv = 0;
  battCell3mv = 0;
  battCell4mv = 0;
  battCell5mv = 0;
  battCell6mv = 0;
  battCell7mv = 0;
  battCell8mv = 0;
}

void recCumminsStatus(CANMessage message){
  static int prevMillis = 0;
  int nowMillis;
  int deltaMillis;
  float tempAmpSeconds;
  float tempWattSeconds;

  if (message.id == CUMMINS_STATUS_ID){
    ResetCumminsTimer(CT_CUMMINS_LOST_DELAY);
    nowMillis = millis();
    battVoltage = (float)((uint16_t)((message.data[1]<<8)|(message.data[0]<<0)))/1000; //voltage
    battCurrent = (float)((int16_t)((message.data[3]<<8)|(message.data[2]<<0)))/10;  //current
    moduleSOCscale = (float)message.data[4];  //SOC
    moduleMaxTemperature = (int8_t)message.data[5];  //temperature
    moduleMinTemperature = moduleMaxTemperature;
    cumminsSystemState = (uint8_t)message.data[6];  //system state
    cumminsDischrgEnabled =  message.data[7]&0x01; //SBX
    DQwallPluggedIn = (message.data[7]>>1)&0x01;  //AC detected
    cumminsInterlockDetected = (message.data[7]>>2)&0x01;  //interlock
    deltaMillis = nowMillis - prevMillis;
    prevMillis = nowMillis;
    tempAmpSeconds = battCurrent * ((float)deltaMillis/1000);
    ampHours = ampHours + (tempAmpSeconds/3600);
    tempWattSeconds = battVoltage * tempAmpSeconds;
    //cumlAmpHrs = cumlAmpHrs + ampHours;
    wattHours = wattHours + (tempWattSeconds/3600);
    return;
  }

}

int CumminsCANok(void){
  if (TimerExpired(&scastCumminsTimers[CT_CUMMINS_LOST_DELAY])){
    return 0;
  }
  else{
    return 1;
  }
}

void ResetCumminsTimer(CUMMINS_TIMERS eCANTimer)
{
  if (eCANTimer < NUM_CUMMINS_TIMERS)
  {
    SetTimerWithMilliseconds(&scastCumminsTimers[eCANTimer], scawCumminsTimers[eCANTimer]);
  }
}
