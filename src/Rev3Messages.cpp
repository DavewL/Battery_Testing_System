 #include "Rev3Messages.h"
 #include "defines.h"
 #include "TickTimer.h"
 #include "CycleTest.h"
 #include "Globals.h"

 static const unsigned int scawCANrev3Timers[NUM_VAL_BMSR3_TIMERS] =
 {
   /* CT_CANR3_LOST_DELAY   */   1000,  //ms -delay before BMS R3 CAN is assumed lost
 };

 static TICK_TIMER_S scastCANrev3Timers[NUM_VAL_BMSR3_TIMERS];

void ResetValR3Timer(VAL_BMSR3_TIMERS eCANTimer);

extern String battCurr2String;
extern String battMinTemp2String;
extern String battMaxTemp2String;
extern String battSOC2String;
extern String battVolt2String;

void initValR3CAN(void){
  unsigned int wIndex;
  for (wIndex = 0; wIndex < ((unsigned int)NUM_VAL_BMSR3_TIMERS); wIndex++){
   RegisterTimer(&scastCANrev3Timers[wIndex]);
   ResetValR3Timer((VAL_BMSR3_TIMERS)wIndex);
  }
  //battType = VALENCE_REV3;
}

 void receiveMessagesRev3(CANMessage message){
   //int tempBitwise = 0;
   //int tempBitwise2 = 0;
   static int prevMillis = 0;
   int nowMillis;
   int deltaMillis;
   float tempAmpSeconds;
   float tempWattSeconds;

   if (message.id == SYS_STATUS_ID){ //-----------------------------------------
     ResetValR3Timer(CT_CANR3_LOST_DELAY);
     if ((message.data[0]&0x07) == 0){   //mux 000
       if ((message.data[0]&0x08) == 1){
         BMSstatus = CHARGE;
       }
       else if ((message.data[0]&0x10) == 1){
        BMSstatus = DISCHARGE;
       }
       else if ((message.data[0]&0x80) == 1){
        BMSstatus = FAULT;
       }
       moduleSOCscale = ((float)(message.data[1])/255)*100;
       //battSOC2String = String::format("%.2f", moduleSOCscale);
       return;
     }
     else if ((message.data[0]&0x07) == 1){  //mux 001
       BMSstatusWord = (unsigned int)(message.data[5]<<24)|(unsigned int)(message.data[4]<<16)|(unsigned int)(message.data[3]<<8)|(unsigned int)(message.data[2]);
       overTemperatureStatus = (message.data[2]&0x03);
       PCBAoverTempStatus = (message.data[2]&0x0C) >> 2;
       battTerminalOverTempStatus = (message.data[2]&0x30) >> 4;
       underTempChargeStatus = (message.data[2]&0xC0) >> 6;

       underTempDischargeStatus = (message.data[3]&0x03);
       overVoltageStatus = (message.data[3]&0x0C) >> 2;
       underVoltageStatus = (message.data[3]&0x30) >> 4;
       overCurrentCharge = (message.data[3]&0xC0) >> 6;

       overCurrentDischarge = (message.data[4]&0x03);

       cellDeltaTempStatus = (message.data[5]&0x03);
       ModuleLostState = (message.data[5]&0x0C) >> 2;
       return;
     }
     else if ((message.data[0]&0x07) == 2){  //mux 002
       return;
     }
     else if ((message.data[0]&0x07) == 3){  //mux 003
       return;
     }
   }
   else if (message.id == SYS_MEAS_ID){ //--------------------------------------
      ResetValR3Timer(CT_CANR3_LOST_DELAY);
      nowMillis = millis();
      battVoltage = (float)((message.data[1]<<8)|message.data[0])*0.032;
      //battVolt2String = String::format("%.3f", battVoltage);

      battCurrent = (float)((int16_t)((message.data[3]<<8)|message.data[2]))*0.1;
      deltaMillis = nowMillis - prevMillis;
      prevMillis = nowMillis;
      tempAmpSeconds = battCurrent * ((float)deltaMillis/1000);
      ampHours = ampHours + (tempAmpSeconds/3600);
      tempWattSeconds = battVoltage * tempAmpSeconds;
      //cumlAmpHrs = cumlAmpHrs + ampHours;
      wattHours = wattHours + (tempWattSeconds/3600);
      //cumlWattHrs = cumlWattHrs + wattHours;
      //battCurr2String = String::format("%.1f", battCurrent);

      maxDischargeCurrent = ((float)((message.data[5]<<8)|message.data[4])*0.1);
      maxRegenCurrent = ((float)((message.data[7]<<8)|message.data[6])*0.1);
      return;
   }
   else if (message.id == SYS_MEAS_TRACE_ID){ //--------------------------------
     ResetValR3Timer(CT_CANR3_LOST_DELAY);
     moduleMinTemperature = (message.data[3])-60;
     moduleMaxTemperature = (message.data[0])-60;
     return;
   }
   else if (message.id == MODULE_INFO_ID){ //-----------------------------------
     ResetValR3Timer(CT_CANR3_LOST_DELAY);
     if (message.data[0] == 1){  //module 1
        if (message.data[1] == 0x0A){  //module index 0
           battCell1mv = (message.data[3]<<8)|(message.data[2]);
           battCell2mv = (message.data[5]<<8)|(message.data[4]);
           battCell3mv = (message.data[7]<<8)|(message.data[6]);
           return;
        }
        else if (message.data[1] == 0x2A){ //Module index 1
           battCell4mv = (message.data[3]<<8)|(message.data[2]);
           battCell5mv = (message.data[5]<<8)|(message.data[4]);
           battCell6mv = (message.data[7]<<8)|(message.data[6]);
           return;
        }
        else if (message.data[1] == 0x4A){ //Module index 2
           battCell7mv = (message.data[3]<<8)|(message.data[2]);
           battCell8mv = (message.data[5]<<8)|(message.data[4]);
           return;
        }
        else if (message.data[1] == 0x04){  //Module index 4
          moduleMaxMvolts = (message.data[3]<<8)|(message.data[2]);
          moduleMinMvolts = (message.data[5]<<8)|(message.data[4]);
          return;
        }
     }
   }
   else if (message.id == CHARGE_INFO_ID){  //----------------------------------
     ResetValR3Timer(CT_CANR3_LOST_DELAY);
     if ((message.data[0] & 0x01) == 1){
       chargeStatus = COMPLETE;
     }
     else{
       chargeStatus = (message.data[0] & 0x06) >> 1;
     }
     BMSchargeCurrSetpoint = ((float)(message.data[2]<<8 | (message.data[1]))*0.1);
     BMSchargeVoltSetpoint = ((float)(message.data[4]<<8 | (message.data[3]))*0.1);
   }
   else if (message.id == CONTACTOR_STATES_ID){
     ResetValR3Timer(CT_CANR3_LOST_DELAY);
     contactorCharge = (message.data[0] & 0x01);
     contactorMain = ((message.data[0] >> 4) & 0x01);
     return;
   }
}

int BMSrev3CANok(void){
  if (TimerExpired(&scastCANrev3Timers[CT_CANR3_LOST_DELAY])){
    return 0;
  }
  else{
    return 1;
  }
}

void ResetValR3Timer(VAL_BMSR3_TIMERS eCANTimer)
{
  if (eCANTimer < NUM_VAL_BMSR3_TIMERS)
  {
    SetTimerWithMilliseconds(&scastCANrev3Timers[eCANTimer], scawCANrev3Timers[eCANTimer]);
  }
}
