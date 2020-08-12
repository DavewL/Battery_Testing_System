#include <carloop.h>
#include "CANtx.h"
#include "Globals.h"
#include "CycleTest.h"

extern Carloop<CarloopRevision2> carloop;
extern stateTestControl testStateCopy;

void sendStatusCAN(void){
  int millisNow2 = millis();
  static int millisPrev2 = 0;
  int millisDelta2 = millisNow2-millisPrev2;
  millisPrev2 = millisNow2;
  CANMessage message;
  message.id = CNTRLR_STATUS_ID;
  message.len = 6;
  message.data[5] = (byte)testStateCopy;
  message.data[4] = (byte)(((heaterStatus<<4)&0b00110000)|((rechargeState<<2)&0b00001100)|(keySignalStatus&0b00000011));
  message.data[3] = (byte)((millisDelta2 & 0xFF000000)>>24);
  message.data[2] = (byte)((millisDelta2 & 0x00FF0000)>>16);
  message.data[1] = (byte)((millisDelta2 & 0x0000FF00)>>8);
  message.data[0] = (byte)(millisDelta2 & 0x000000FF);
  carloop.can().transmit(message);

}
