#include "CANinit.h"
#include "Rev3Messages.h"
#include "CumminsCAN.h"
#include "defines.h"
#include "Globals.h"

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
    else{
      carloop.setCANSpeed(VALENCE_BAUD);        //must set can speed before enabling bus!
      //initValR3CAN();
    }
    carloop.enableCAN();                        //enable canbus
}
