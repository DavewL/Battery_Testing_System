//Invntus Message Protocol
#include "Invnts80AhCAN.h"
#include "defines.h"
#include "Globals.h"
#include "TickTimer.h"

extern Carloop<CarloopRevision2> carloop;

static const unsigned int scawInvntsTimers[NUM_INVNTS_TIMERS] =
{
  /* CT_INVNTS_LOST_DELAY   */   2000,  //ms -delay before INVNTS CAN is assumed lost
};

static TICK_TIMER_S scastInvntsTimers[NUM_INVNTS_TIMERS];

int cellVreads[8] = {0,0,0,0,0,0,0,0};

void ResetInvntsTimer(INVNTS_TIMERS eCANTimer);
void HandleCellVReads(int cellIndex);


void initInvnts80AhCAN(void){
  unsigned int wIndex;  
  for (wIndex = 0; wIndex < ((unsigned int)NUM_INVNTS_TIMERS); wIndex++){
   RegisterTimer(&scastInvntsTimers[wIndex]);
   ResetInvntsTimer((INVNTS_TIMERS)wIndex);
  }
}

void recInvnts80AhStatus(CANMessage message){
  static int prevMillis = 0;
  int nowMillis;
  int deltaMillis;
  float tempAmpSeconds;
  float tempWattSeconds;

  if (message.id == INVNTS_TPDO3_ID){
    ResetInvntsTimer(CT_INVNTS_LOST_DELAY);
    nowMillis = millis();
    if (((message.data[1]) == INVNTS_MUX_1) && (message.data[0] == 1)){  //data[0] is module number
      moduleSOCscale = (float)message.data[2];  //SOC %
      InvntsSOH = (int)message.data[3];
      maxDischargeCurrent = (float)((uint16_t)((message.data[5]<<8)|(message.data[4]<<0)))/100;
      BMSchargeCurrSetpoint = (float)((uint16_t)((message.data[7]<<8)|(message.data[6]<<0)))/100;
      //Serial.println(BMSchargeCurrSetpoint);
    }
    if (((message.data[1]) == INVNTS_MUX_2) && (message.data[0] == 1)){  //data[0] is module number
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
    if (((message.data[1]) == INVNTS_MUX_3) && (message.data[0] == 1)){  //data[0] is module number
      InvntsCANRec = 1;
      /* -- charge faults ------------------------------------------------*/
      overTempChargeStatus = ((message.data[2]&0x01))*3;        //0000 0001
      underTempChargeStatus = ((message.data[2]&0x02) >> 1)*3;  //0000 0010
      overCurrentCharge = ((message.data[2]&0x04) >> 2)*3;      //0000 0100
      overVoltageStatus = ((message.data[2]&0x08) >> 3)*3;      //0000 1000
      shortCircuitStatus = ((message.data[2]&0x016) >> 4)*3;    //0001 0000
      otherChargeFaultStatus = ((message.data[2]&0x032) >> 5)*3;//0010 0000

      ModuleLostState = ((message.data[3]&0x016) >> 4)*3;       //0001 0000
      internalCommStatus = ((message.data[3]&0x032) >> 5)*3;    //0010 0000

      /* -- discharge faults ---------------------------------------------*/
      overTempDischargeStatus = ((message.data[4]&0x01))*3;           //0000 0001
      underTempDischargeStatus = ((message.data[4]&0x02) >> 1)*3;     //0000 0010
      overCurrentDischarge = ((message.data[4]&0x04) >> 2)*3;         //0000 0100
      underVoltageStatus = ((message.data[4]&0x08) >> 3)*3;           //0000 1000
      if (shortCircuitStatus == 0){
        shortCircuitStatus = ((message.data[4]&0x016) >> 4)*3;        //0001 0000
      }
      otherDischargeFaultStatus = ((message.data[4]&0x032) >> 5)*3;   //0010 0000
      if (ModuleLostState == 0){
        ModuleLostState = ((message.data[5]&0x16) >> 4)*3;            //0001 0000
      }
      if (internalCommStatus){
        internalCommStatus = ((message.data[5]&0x032) >> 5)*3;        //0010 0000
      }

      if ((message.data[2] != 0) || (message.data[3] != 0)){
        BMSstatusWord = (uint16_t)((message.data[3]<<8)|(message.data[2]));
      }
      else if ((message.data[4] != 0 ) || (message.data[5] != 0)){
        BMSstatusWord = (uint16_t)((message.data[5]<<8)|(message.data[4]));
      }

      //moduleMaxTemperature = ((float)((int16_t)((message.data[7]<<8)|(message.data[6]<<0)))/10)-273.15;
      //moduleMinTemperature = moduleMaxTemperature;
    }
  }
  else if (message.id == INVNTS_CELL_VOLTS_ID){
    ResetInvntsTimer(CT_INVNTS_LOST_DELAY);
    nowMillis = millis();
    if (message.data[3] == CELL_1_MUX){
      battCell1mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(0);
    }
    else if (message.data[3] == CELL_2_MUX){
      battCell2mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(1);
    }
    else if (message.data[3] == CELL_3_MUX){
      battCell3mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(2);
    }
    else if (message.data[3] == CELL_4_MUX){
      battCell4mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(3);
    }
    else if (message.data[3] == CELL_5_MUX){
      battCell5mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(4);
    }
    else if (message.data[3] == CELL_6_MUX){
      battCell6mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(5);
    }
    else if (message.data[3] == CELL_7_MUX){
      battCell7mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(6);
    }
    else if (message.data[3] == CELL_8_MUX){
      battCell8mv = (uint16_t)((message.data[5]<<8)|(message.data[4]<<0));
      HandleCellVReads(7);
    }
    
  }
  else if (message.id == INVNTS_SDO_RESP_ID){
    ResetInvntsTimer(CT_INVNTS_LOST_DELAY);
    nowMillis = millis();
    if ((message.data[0] == 0x4B)&&(message.data[1]==0x01)&&(message.data[2]==0xC1)){
      if (message.data[3] == INVNTS_VOLTS_SUBINDEX){
        battVoltage = (float)((uint16_t)((message.data[5]<<8)|(message.data[4]<<0)))/1000; //voltage
      }
      else if (message.data[3]== INVNTS_CURRENT_SUBINDEX){
        battCurrent = (float)((int16_t)((message.data[5]<<8)|(message.data[4]<<0)))/100;  //current
        if (battVoltage > 0){
          deltaMillis = nowMillis - prevMillis;
          prevMillis = nowMillis;
          tempAmpSeconds = battCurrent * ((float)deltaMillis/1000);
          ampHours = ampHours + (tempAmpSeconds/3600);
          tempWattSeconds = battVoltage * tempAmpSeconds;
          //cumlAmpHrs = cumlAmpHrs + ampHours;
          wattHours = wattHours + (tempWattSeconds/3600);
        } 
      }
      else if (message.data[3]==INVNTS_HEATER_STATUS_SUBINDEX){
        InvntsHeaterStat = (uint8_t)message.data[4];
      }
      else if (message.data[3] == INVNTS_CHRG_STATUS_SUBINDEX){
        if (message.data[4] == 32){
          chargeStatus = COMPLETE;
        }
        else{
          chargeStatus = MAIN;
        }
      }
      else if (message.data[3] == INVNTS_MIN_CELL_TEMP){
        moduleMinTemperature = ((float)((int16_t)((message.data[5]<<8)|(message.data[4]<<0)))/10)-273.15;
      }
      else if (message.data[3] == INVNTS_MAX_CELL_TEMP){
        moduleMaxTemperature = ((float)((int16_t)((message.data[5]<<8)|(message.data[4]<<0)))/10)-273.15;
      }
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

void InvntsSDOReadReq(int sub_index){
  CANMessage message;
  message.id = 0x631;
  message.len = 4;
  message.data[0] = 0x40;
  message.data[1] = 0x01;  //pack number
  message.data[2] = 0xC1;
  message.data[3] = sub_index;

  carloop.can().transmit(message);
}

void InvntsSDOWriteReq(int sub_index, int wr_data){
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

void HandleCellVReads(int cellIndex){
  cellVreads[cellIndex] = 1;  //update this cells read status array value to 1

  int allCellsUpdated = 1;    //used to trigger min/max checks when all cells have been read

  for (int i = 0; i<8; i++){    //check to see if all cells have been read
    if (cellVreads[i] == 0){
      allCellsUpdated = 0;
    }
  }

  if (allCellsUpdated == 1){        // if all cell have been read, update the min/max values
    moduleMaxMvolts = battCell1mv;
    moduleMinMvolts = battCell1mv;
    if (battCell2mv > moduleMaxMvolts) {
      moduleMaxMvolts = battCell2mv;
    }
    if (battCell2mv < moduleMinMvolts){
      moduleMinMvolts = battCell2mv;
    }
    if (battCell3mv > moduleMaxMvolts) {
      moduleMaxMvolts = battCell3mv;
    }
    if (battCell3mv < moduleMinMvolts){
      moduleMinMvolts = battCell3mv;
    }
    if (battCell4mv > moduleMaxMvolts) {
      moduleMaxMvolts = battCell4mv;
    }
    if (battCell4mv < moduleMinMvolts){
      moduleMinMvolts = battCell4mv;
    }
    if (battCell5mv > moduleMaxMvolts) {
      moduleMaxMvolts = battCell5mv;
    }
    if (battCell5mv < moduleMinMvolts){
      moduleMinMvolts = battCell5mv;
    }
    if (battCell6mv > moduleMaxMvolts) {
      moduleMaxMvolts = battCell6mv;
    }
    if (battCell6mv < moduleMinMvolts){
      moduleMinMvolts = battCell6mv;
    }
    if (battCell7mv > moduleMaxMvolts) {
      moduleMaxMvolts = battCell7mv;
    }
    if (battCell7mv < moduleMinMvolts){
      moduleMinMvolts = battCell7mv;
    }
    if (battCell8mv > moduleMaxMvolts) {
      moduleMaxMvolts = battCell8mv;
    }
    if (battCell8mv < moduleMinMvolts){
      moduleMinMvolts = battCell8mv;
    }
    for (int n = 0; n<8; n++){  //reset the read status array to 0's
      cellVreads[n] = 0;
    }
  }
}
