#include "DeltaQ_CANopen.h"
#include "defines.h"
#include "TickTimer.h"
//#include "CycleTest.h"
#include "Globals.h"

extern Carloop<CarloopRevision2> carloop;

//static const unsigned int scawDqCANopenTimers[NUM_DQCAN_TIMERS] =
//{
//  /* CT_DQCANOPEN_LOST_DELAY   */   1000,  //ms -delay before deltaQ CANopen CAN is assumed lost
//};

//static TICK_TIMER_S scastDqCANopenTimers[NUM_DQCAN_TIMERS];

//void ResetDQCANopenTimer(DQCAN_TIMERS1 eCANTimer);

//void initDQCANopenRx(void){
  //unsigned int wIndex;
  //for (wIndex = 0; wIndex < ((unsigned int)NUM_DQCAN_TIMERS); wIndex++){
    //RegisterTimer(&scastDqCANopenTimers[wIndex]);
    //ResetDQCANopenTimer((DQCAN_TIMERS1)wIndex);
//  }
//}

void receiveMesDQCANopen(CANMessage message){
  //int tempBitwise = 0;
  //int tempBitwise2 = 0;

  if (message.id == DQ_HB_ID){ //----------------------DELTA-Q CANOPEN HEARTBEAT----------
    //ResetDQCANopenTimer(CT_DQCANOPEN_LOST_DELAY);
    if (message.data[0] == 0x7F){ //----DELTA-Q IS IN PRE-OP STATE
      DQoperational = 0;
    }
    else if (message.data[0] == 0x0A){ //-----DELTA-Q IS IN OPERATION statement
      DQoperational = 1;
    }
    return;
  }
  else if (message.id == DQ_TPDO1_ID){ //----------------------DELTA-Q TxPDO 1
    DQwallPluggedIn = (message.data[4] >> 4) & 0x01;
    DQcurrent = (float)((message.data[1]<<8)|(message.data[0]<<0))*0.003906;
    DQvoltage = (float)((message.data[3]<<8)|(message.data[2]<<0))*0.003906;

    return;
  }
  else if (message.id == DQ_IC1200_TPDO1_ID){ //----------------DELTA-Q IC1200 TxPDO 1
    IC1200wallPluggedIn = (message.data[4] >> 4) & 0x01;
    IC1200current = (float)((message.data[1]<<8)|(message.data[0]<<0))*0.003906;
    IC1200voltage = (float)((message.data[3]<<8)|(message.data[2]<<0))*0.003906;

  }
}

//int DQCANpenok(void){
//  if (TimerExpired(&scastDqCANopenTimers[CT_DQCANOPEN_LOST_DELAY])){
//    return 0;
//  }
//  else{
//    return 1;
//  }
//}

//void ResetDQCANopenTimer(DQCAN_TIMERS1 eCANTimer)
//{
//  if (eCANTimer < NUM_DQCAN_TIMERS)
//  {
//    SetTimerWithMilliseconds(&scastDqCANopenTimers[eCANTimer], scawDqCANopenTimers[eCANTimer]);
//  }
//}

//-----------------------------------------------------TX MESSAGES----------------------------------
void transmitDQGoOp(void){
  CANMessage message;
  message.id = DQ_REPLY_HB_ID;
  message.len = 1;
  message.data[0] = 0x05;
  carloop.can().transmit(message);
}

void transmitRPDO1(void){
  uint16_t bitsVoltage = (uint16_t)(setDQVoltage*256);  //setDQVoltage is float
  uint16_t bitsCurrent = (uint16_t)(setDQCurrent*16);   //setDQCurrent is float

  CANMessage message;
  message.id = DQ_RPDO1_ID;
  message.len = 8;
  message.data[0] = (byte)(DQchargerEnable & 0x0001); //charge control
  message.data[1] = 0x01; //battery SOC
  message.data[2] = (byte)(bitsVoltage >> 0) & 0xFF;
  message.data[3] = (byte)(bitsVoltage >> 8) & 0xFF;
  message.data[4] = (byte)(bitsCurrent >> 0) & 0xFF;
  message.data[5] = (byte)(bitsCurrent >> 8) & 0xFF;
  message.data[6] = 0xFF;
  message.data[7] = 0xFF;

  carloop.can().transmit(message);
}

void transmitIC1200VoltsAmps(void){
  uint16_t bitsVoltage = (uint16_t)(setIC1200Voltage*256);
  uint16_t bitsCurrent = (uint16_t)(setIC1200Current*16);

  CANMessage message;
  message.id = DQ_IC1200_RPDO1_ID;
  message.len = 8;
  message.data[0] = (byte)(IC1200Enable & 0x0001); //charge control
  message.data[1] = 0x01; //battery SOC
  message.data[2] = (byte)(bitsVoltage >> 0) & 0xFF;
  message.data[3] = (byte)(bitsVoltage >> 8) & 0xFF;
  message.data[4] = (byte)(bitsCurrent >> 0) & 0xFF;
  message.data[5] = (byte)(bitsCurrent >> 8) & 0xFF;
  message.data[6] = 0xFF;
  message.data[7] = 0xFF;

  carloop.can().transmit(message);
}


void transmitNMT(void){
  CANMessage message;
  message.id = DQ_NMT_ID;
  message.len = 2;
  message.data[0] = 0x01;
  message.data[1] = 0x0A;
carloop.can().transmit(message);
}

void transmitDQSDOready(void){
  CANMessage message;
  message.id = DQ_SDO_ID;
  message.len = 8;
  message.data[0] = 0x2F;
  message.data[1] = 0x00;
  message.data[2] = 0x60; //SDO to object 0x6000
  message.data[3] = 0x00;
  message.data[4] = 0x01; //battery ready bit
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;

  carloop.can().transmit(message);

}
