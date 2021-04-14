#include "CANinit.h"
#include "Rev3Messages.h"
#include "CumminsCAN.h"
#include "defines.h"
#include "Globals.h"
#include "InvntsOldCAN.h"
#include "Invnts60AhCAN.h"
#include "Invnts80AhCAN.h"

#include <carloop.h>

Carloop<CarloopRevision2> carloop;

//-----------------INITIALIZE CANBUS---------------------------------------
void initCAN(void){
    //setup CANbus interface
    carloop.begin(CARLOOP_CAN);
    if (battType==VALENCE_REV3){
      carloop.setCANSpeed(VALENCE_BAUD);        //must set can speed before enabling bus!
      initValR3CAN();
    }
    else if (battType == CUMMINS_REV1){
      carloop.setCANSpeed(CUMMINS_BAUD);        //must set can speed before enabling bus!
      initCumminsCAN();
    }
    else if (battType == INVNTS_OLD){
      carloop.setCANSpeed(INVNTS_BAUD);        //must set can speed before enabling bus!
      initInvntsOldCAN();
    }
    else if (battType == INVNTS_60AH){
      carloop.setCANSpeed(INVNTS_BAUD);        //must set can speed before enabling bus!
      initInvnts60AhCAN();
    }
    else if (battType == INVNTS_80AH){
      carloop.setCANSpeed(INVNTS_BAUD);        //must set can speed before enabling bus!
      initInvnts80AhCAN();
    }

    else{
      carloop.setCANSpeed(VALENCE_BAUD);        //must set can speed before enabling bus!
      //initValR3CAN();
    }
    carloop.enableCAN();                        //enable canbus
}
