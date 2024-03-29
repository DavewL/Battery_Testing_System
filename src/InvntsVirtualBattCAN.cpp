//Invntus Message Protocol
#include "InvntsVirtualBattCAN.h"
#include "defines.h"
#include "Globals.h"
#include "TickTimer.h"

extern Carloop<CarloopRevision2> carloop;

static const unsigned int scawInvntsVirtualBattTimers[NUM_INVNTS_VIRT_BATT_TIMERS] =
{
  /* CT_INVNTS_VIRT_BATT_LOST_DELAY   */   6000,  //ms -delay before INVNTS CAN is assumed lost
};

static TICK_TIMER_S scastInvntsVirtualBattTimers[NUM_INVNTS_VIRT_BATT_TIMERS];

void ResetInvntsTimer(INVNTS_VIRT_BATT_TIMERS eCANTimer);

void initInvntsVirtualBattCAN(void){
  unsigned int wIndex;  
  for (wIndex = 0; wIndex < ((unsigned int)NUM_INVNTS_VIRT_BATT_TIMERS); wIndex++){
   RegisterTimer(&scastInvntsVirtualBattTimers[wIndex]);
   ResetInvntsTimer((INVNTS_VIRT_BATT_TIMERS)wIndex);
  }
}

void recInvntsVirtualBattStatus(CANMessage message){
  static int prevMillis = 0;
  int nowMillis;
  int deltaMillis;
  float tempAmpSeconds;
  float tempWattSeconds;

  if (message.id == INVNTS_TPDO1_ID){
    ResetInvntsTimer(CT_INVNTS_VIRT_BATT_LOST_DELAY);
    nowMillis = millis();
    //moduleSOCscale = (float)message.data[1];  //SOC %
  }

  else if (message.id == INVNTS_TPDO2_ID){
    ResetInvntsTimer(CT_INVNTS_VIRT_BATT_LOST_DELAY);
    nowMillis = millis();

    //battVoltage = (float)((uint16_t)((message.data[1]<<8)|(message.data[0]<<0)))*0.001; //voltage
    battCurrent = (float)((int16_t)((message.data[3]<<8)|(message.data[2]<<0)))/10;  //current

    if (battVoltage > 0){
        deltaMillis = nowMillis - prevMillis;
        prevMillis = nowMillis;
        tempAmpSeconds = battCurrent * ((float)deltaMillis/1000);
        ampHours = ampHours + (tempAmpSeconds/3600);
        tempWattSeconds = battVoltage * tempAmpSeconds;
        //cumlAmpHrs = cumlAmpHrs + ampHours;
        wattHours = wattHours + (tempWattSeconds/3600);
      }
    maxDischargeCurrent = (float)((uint16_t)((message.data[5]<<8)|(message.data[4]<<0)))/10;     
  }

  else if (message.id == INVNTS_TPDO4_ID){
    ResetInvntsTimer(CT_INVNTS_VIRT_BATT_LOST_DELAY);
    nowMillis = millis();

    InvntsSOH = (int)message.data[0];   

    //Serial.println(BMSchargeCurrSetpoint);
   
    BMSstatusWord = message.data[3];

    InvntsCANRec = 1;
    /* -- charge faults ------------------------------------------------*/
    overTempChargeStatus = ((message.data[4]&0x01))*3;        //0000 0001
    underTempChargeStatus = ((message.data[4]&0x02) >> 1)*3;  //0000 0010
    overCurrentCharge = ((message.data[4]&0x04) >> 2)*3;      //0000 0100
    overVoltageStatus = ((message.data[4]&0x08) >> 3)*3;      //0000 1000
    shortCircuitStatus = ((message.data[4]&0x016) >> 4)*3;    //0001 0000
    otherChargeFaultStatus = ((message.data[4]&0x032) >> 5)*3;//0010 0000

    ModuleLostState = ((message.data[5]&0x016) >> 4)*3;       //0001 0000
    internalCommStatus = ((message.data[5]&0x032) >> 5)*3;    //0010 0000

    /* -- discharge faults ---------------------------------------------*/
    overTempDischargeStatus = ((message.data[6]&0x01))*3;           //0000 0001
    underTempDischargeStatus = ((message.data[6]&0x02) >> 1)*3;     //0000 0010
    overCurrentDischarge = ((message.data[6]&0x04) >> 2)*3;         //0000 0100
    underVoltageStatus = ((message.data[6]&0x08) >> 3)*3;           //0000 1000
    if (shortCircuitStatus == 0){
      shortCircuitStatus = ((message.data[6]&0x016) >> 4)*3;        //0001 0000
    }
    otherDischargeFaultStatus = ((message.data[6]&0x032) >> 5)*3;   //0010 0000
    if (ModuleLostState == 0){
      ModuleLostState = ((message.data[7]&0x16) >> 4)*3;            //0001 0000
    }
    if (internalCommStatus){
      internalCommStatus = ((message.data[7]&0x032) >> 5)*3;        //0010 0000
    }
  }

  else if (message.id == INVNTS_TPDO3_ID){
      ResetInvntsTimer(CT_INVNTS_VIRT_BATT_LOST_DELAY);
      nowMillis = millis();

      BMSchargeCurrSetpoint = (float)((uint16_t)((message.data[5]<<8)|(message.data[4]<<0)))/10;
      BMSchargeVoltSetpoint = (float)((uint16_t)((message.data[7]<<8)|(message.data[6]<<0)))*0.001; //max charge voltage
  }
  else if (message.id == INVNTS_TPDO5_ID){
      ResetInvntsTimer(CT_INVNTS_VIRT_BATT_LOST_DELAY);
      nowMillis = millis();

      maxRegenCurrent = (float)((uint16_t)((message.data[1]<<8)|(message.data[0]<<0)))/10;
      
      moduleMinMvolts = (float)((uint16_t)((message.data[3]<<8)|(message.data[2]<<0)))*0.001; //cell min voltage
      moduleMaxMvolts = (float)((uint16_t)((message.data[5]<<8)|(message.data[4]<<0)))*0.001; //cell max voltage
      
      battCell1mv = (int)(moduleMinMvolts*1000);
      battCell2mv = (int)(moduleMaxMvolts*1000);

  }
  else if (message.id == INVNTS_TPDO6_ID){
      ResetInvntsTimer(CT_INVNTS_VIRT_BATT_LOST_DELAY);
      nowMillis = millis();

      battVoltage = (float)((uint16_t)((message.data[1]<<8)|(message.data[0]<<0)))*0.001; //voltage

      moduleSOCscale = (float)message.data[2];  //SOC %
      
      //moduleMaxTemperature = ((float)((int16_t)((message.data[4]<<8)|(message.data[3]<<0)))*.125);// -273.15;
      //moduleMinTemperature = moduleMaxTemperature;
  }

  else if (message.id == INVNTS_SDO_RESP_ID){
    ResetInvntsTimer(CT_INVNTS_VIRT_BATT_LOST_DELAY);
    nowMillis = millis();
    
    if ((message.data[2]<<8)|(message.data[1]) == INVNTS_MAX_CELL_TEMP_INDEX){
      if (message.data[3] == INVNTS_MAX_CELL_TEMP_SUBINDEX){
        moduleMaxTemperature = ((float)((int16_t)((message.data[5]<<8)|(message.data[4]<<0)))*.125);
      }
    }
    if ((message.data[2]<<8)|(message.data[1]) == INVNTS_INTERNAL_STAT_INDEX){    //(message.data[0] == 0x4B)&&(message.data[1]==0x01)&&(message.data[2]==0xC1)){
      if (message.data[3] == INVNTS_ATSAM_VER_SUBINDEX){
        Invnts_ATSAMfirmwareLetterRev = (uint8_t)(message.data[5]);
        Invnts_ATSAMfirmwareMajorRev = (uint8_t)(message.data[7]);
        Invnts_ATSAMfirmwareMinorRev = (uint8_t)(message.data[6]);
      }
      else if (message.data[3] == INVNTS_BQ80_VER_SUBINDEX){
        Invnts_BQ80firmwareLetterRev = (uint8_t)(message.data[5]);
        Invnts_BQ80firmwareMajorRev = (uint8_t)(message.data[7]);
        Invnts_BQ80firmwareMinorRev = (uint8_t)(message.data[6]);
      }
    }
    if ((message.data[2]<<8)|(message.data[1]) == INVNTS_MIN_CELL_TEMP_INDEX){
      if (message.data[3] == INVNTS_MIN_CELL_TEMP_SUBINDEX){
        moduleMinTemperature = ((float)((int16_t)((message.data[5]<<8)|(message.data[4]<<0)))*.125);
      }
    }
  }
}

int InvntsVirtualBattCANok(void){
  if (TimerExpired(&scastInvntsVirtualBattTimers[CT_INVNTS_VIRT_BATT_LOST_DELAY])){
    return 0;
  }
  else{
    return 1;
  }
}

void ResetInvntsTimer(INVNTS_VIRT_BATT_TIMERS eCANTimer)
{
  if (eCANTimer < NUM_INVNTS_VIRT_BATT_TIMERS)
  {
    SetTimerWithMilliseconds(&scastInvntsVirtualBattTimers[eCANTimer], scawInvntsVirtualBattTimers[eCANTimer]);
  }
}

void InvntsVirtualBattSDOReadReq(int index, int sub_index){
  CANMessage message;
  message.id = 0x631;
  message.len = 8;
  message.data[0] = 0x40;
  message.data[1] = (byte)(index >> 0) & 0xFF;
  message.data[2] = (byte)(index >> 8) & 0xFF;
  message.data[3] = (byte)sub_index;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;
  carloop.can().transmit(message);
}

void InvntsVirtualBattSDOWriteReq(int sub_index, int wr_data){
  CANMessage message;
  message.id = 0x631;
  message.len = 4;
  message.data[0] = 0x2B;
  message.data[1] = 0x01;  //pack number
  message.data[2] = 0xC1;
  message.data[3] = sub_index;
  message.data[4] = (byte)(wr_data >> 0) & 0xFF;
  message.data[5] = (byte)(wr_data >> 8) & 0xFF;
  carloop.can().transmit(message);
}
