//Invntus Message Protocol
#include "Invnts80AhCAN.h"
#include "defines.h"
#include "Globals.h"
#include "TickTimer.h"

static const unsigned int scawInvntsTimers[NUM_INVNTS_TIMERS] =
{
  /* CT_INVNTS_LOST_DELAY   */   1000,  //ms -delay before INVNTS CAN is assumed lost
};

static TICK_TIMER_S scastInvntsTimers[NUM_INVNTS_TIMERS];

void ResetInvntsTimer(INVNTS_TIMERS eCANTimer);


void initInvnts80AhCAN(void){
  unsigned int wIndex;
  for (wIndex = 0; wIndex < ((unsigned int)NUM_INVNTS_TIMERS); wIndex++){
   RegisterTimer(&scastInvntsTimers[wIndex]);
   ResetInvntsTimer((INVNTS_TIMERS)wIndex);
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

void recInvntsStatus(CANMessage message){
  static int prevMillis = 0;
  int nowMillis;
  int deltaMillis;
  float tempAmpSeconds;
  float tempWattSeconds;

  if (message.id == INVNTS_STATUS_ID){
    ResetInvntsTimer(CT_INVNTS_LOST_DELAY);
    nowMillis = millis();
    battVoltage = (float)((uint16_t)((message.data[1]<<8)|(message.data[0]<<0)))/1000; //voltage
    battCurrent = (float)((int16_t)((message.data[3]<<8)|(message.data[2]<<0)))/10;  //current
    moduleSOCscale = (float)message.data[4];  //SOC
    moduleMaxTemperature = (int8_t)message.data[5];  //temperature
    moduleMinTemperature = moduleMaxTemperature;
    InvntsSystemState = (uint8_t)message.data[6];  //system state
    InvntsDischrgEnabled =  message.data[7]&0x01; //SBX
    DQwallPluggedIn = (message.data[7]>>1)&0x01;  //AC detected
    InvntsInterlockDetected = (message.data[7]>>2)&0x01;  //interlock
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

int Invnts80AhCANok(void){
  if (TimerExpired(&scastInvntsTimers[CT_INVNTS_LOST_DELAY])){
    return 0;
  }
  else{
    return 1;
  }
}

void ResetInvntsTimer(INVNTS_TIMERS eCANTimer)
{
  if (eCANTimer < NUM_INVNTS_TIMERS)
  {
    SetTimerWithMilliseconds(&scastInvntsTimers[eCANTimer], scawInvntsTimers[eCANTimer]);
  }
}
