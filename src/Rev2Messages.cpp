#include "Rev2Messages.h"

//-----Global variables-------------------------
extern float moduleSOCscale;
extern float battVoltage;
extern float battCurrent;
extern float maxDischargeCurrent;
extern float maxRegenCurrent;
extern int chargeStatus;
extern int BMSstatus;
extern int critDischAlarm;
extern int warningFlags;
extern float moduleTemperature;
extern int battCell1mv;
extern int battCell2mv;
extern int battCell3mv;
extern int battCell4mv;
extern int battCell5mv;
extern int battCell6mv;
extern int battCell7mv;
extern int battCell8mv;
extern int battSN;

//----------------------------------------------

void receiveMessagesRev2(CANMessage message){

  int tempBitwise = 0;
  int tempBitwise2 = 0;

  if(message.id == BMS_STATUS_ID){
      //SOCpercent = (int)message.data[0];
      BMSstatus = (message.data[1] & 0x03);
      chargeStatus = ((int)message.data[1] & 0x0C) >> 2;
      critDischAlarm = (message.data[2]&0x10) >> 4;  //asserted when voltage is critically low
  }
  else if(message.id == BATT_INFO_ID){
      battVoltage = message.data[0];
      tempBitwise = ((~(message.data[2])&0x80)|(message.data[2]&0x7F))&0xFFFF; //deal with BMS's screwball bit manipulation for negative numbers
      tempBitwise2 = (tempBitwise<<8)|(message.data[1]&0xFF);
      if(tempBitwise>>7){
        tempBitwise2 = tempBitwise2|0xFFFF0000;
      }
      battCurrent = tempBitwise2;
  }
  else if(message.id == BATT_CELL_VOLTS_0_ID){
      if(message.data[1] == 0){
          battCell1mv = (message.data[2]<<8)|(message.data[3]&0xFF);
          battCell2mv = (message.data[4]<<8)|(message.data[5]&0xFF);
          battCell3mv = (message.data[6]<<8)|(message.data[7]&0xFF);
      }
      else if (message.data[1] == 1){
          battCell7mv = (message.data[2]<<8)|(message.data[3]&0xFF);
          battCell8mv = (message.data[4]<<8)|(message.data[5]&0xFF);
      }
  }
  else if(message.id == BATT_CELL_VOLTS_1_ID){
      if(message.data[1] == 0){
          battCell4mv = (message.data[2]<<8)|(message.data[3]&0xFF);
          battCell5mv = (message.data[4]<<8)|(message.data[5]&0xFF);
          battCell6mv = (message.data[6]<<8)|(message.data[7]&0xFF);
      }
  }
  else if(message.id == BATT_MODULE_CURR_ID){
      //moduleCurrent = (message.data[2]<<8)|(message.data[3]&0xFF);
  }
  else if(message.id == MODULE_TEMPERATURE_ID){
      moduleTemperature = (float)((message.data[2]<<8)|(message.data[3]&0xFF))/100;

  }
  else if(message.id == MODULE_SOC_ID){
      moduleSOCscale = ((float)(message.data[1])/255)*100;
  }
  else if(message.id == MODULE_SN_FW){
      //if(message.data[0] == 0x01){
      //    if(message.data[1] == 0x03){
      //        moduleSN;
      //        moduleYear;
      //        moduleWeek;
      //    }
      //}
  }
}
