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
}

void recInvntsStatus(CANMessage message){
  static int prevMillis = 0;
  int nowMillis;
  int deltaMillis;
  float tempAmpSeconds;
  float tempWattSeconds;

  if (message.id == INVNTS_TPDO3_ID){
    ResetInvntsTimer(CT_INVNTS_LOST_DELAY);
    nowMillis = millis();
    if (((message.data[1]) == INVNTS_MUX_1) && (message.data[0] == 0)){  //data[0] is module number
      moduleSOCscale = (float)message.data[2];  //SOC %
      InvntsSOH = (int)message.data[3];
      maxDischargeCurrent = (float)((uint16_t)((message.data[5]<<8)|(message.data[4]<<0)))/100;
      BMSchargeCurrSetpoint = (float)((uint16_t)((message.data[7]<<8)|(message.data[6]<<0)))/100;
    }
    if (((message.data[1]) == INVNTS_MUX_2) && (message.data[0] == 0)){  //data[0] is module number
      maxRegenCurrent = (float)((uint16_t)((message.data[3]<<8)|(message.data[2]<<0)))/100;
      if (message.data[4] == 5){
        BMSstatus = DISCHARGE;
      }
      else if (message.data[4] == 6){
        BMSstatus = CHARGE;
      }
      else if (message.data[4] == 7){
        BMSstatus = CHARGE;
      }
      else if (message.data[4] == 4){
        BMSstatus = STANDBY;
      }
    }
    if (((message.data[1]) == INVNTS_MUX_3) && (message.data[0] == 0)){  //data[0] is module number
      moduleMaxTemperature = ((float)((int16_t)((message.data[7]<<8)|(message.data[6]<<0)))/10)-273.15;
      moduleMinTemperature = moduleMaxTemperature;
    }
  }
  if (message.id == INVNTS_CELL_VOLTS_ID){
    ResetInvntsTimer(CT_INVNTS_LOST_DELAY);
    nowMillis = millis();
    if (message.data[3] == CELL_1_MUX){
      battCell1mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
    else if (message.data[3] == CELL_2_MUX){
      battCell2mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
    else if (message.data[3] == CELL_3_MUX){
      battCell3mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
    else if (message.data[3] == CELL_4_MUX){
      battCell4mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
    else if (message.data[3] == CELL_5_MUX){
      battCell5mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
    else if (message.data[3] == CELL_6_MUX){
      battCell6mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
    else if (message.data[3] == CELL_7_MUX){
      battCell7mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
    else if (message.data[3] == CELL_8_MUX){
      battCell8mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
    }
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

int InvntsReqModVolts(void){

}
int InvntsReqModCurrent(void){

}
int InvntsReqHeaterStatus(void){

}
battVoltage = (float)((uint16_t)((message.data[1]<<8)|(message.data[0]<<0)))/1000; //voltage
    battCurrent = (float)((int16_t)((message.data[3]<<8)|(message.data[2]<<0)))/10;  //current
    
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