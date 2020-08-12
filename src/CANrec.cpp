#include "CANrec.h"
#include <carloop.h>
#include "Rev2Messages.h"
#include "Rev3Messages.h"
#include "CSM_Temp_CAN_Rec.h"
#include "DeltaQ_CANopen.h"
#include "CumminsCAN.h"
#include "Globals.h"
#include "defines.h"

int msgRec = 1;

extern Carloop<CarloopRevision2> carloop;

//-------------SERVICE NEW CAN MESSAGES-------------------------------------
int canReceiveMessage(){
  CANMessage message;
  int tempBitwise = 0;
  int tempBitwise2 = 0;
  static int timePrevMsg = 0;
  int timeNow = Time.now();

  if(timePrevMsg == 0){
      timePrevMsg = timeNow;
  }

  if(carloop.can().receive(message)){  //if new can message received, decode and reset counter
    msgRec = 1;
    //receiveMessagesRev2(message);    //DECODE REV 2 BMS MESSAGES
    if (battType == VALENCE_REV3){
      receiveMessagesRev3(message);   //DECODE REV 3 BMS MESSAGES
      receiveCSMtemps(message);       //DECODE CSM TEMPERATURE MESSAGES
      receiveMesDQCANopen(message);   //DECODE DELTA-Q CANOPEN MESSAGES
    }
    else if (battType == CUMMINS_REV1){
      recCumminsStatus(message);
    }
    //receiveMessagesTraction();  //DECODE TRACTION MESSAGES
    //receiveMessagesVCU();       //DECODE VCU MESSAGES

  }
  else{
    if ((timeNow - timePrevMsg) > CAN_LOST_TIME){ //if more than CAN_LOST_TIME seconds has elapsed since last message
      msgRec = 0;
    }
  }
  return msgRec;
}

//------------------RETURN IF CANBUS IS OK----------------------------------
bool CANok(void)
{
    return (bool)msgRec;
}
