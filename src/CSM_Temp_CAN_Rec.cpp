#include "CSM_Temp_CAN_Rec.h"
#include "defines.h"
#include "TickTimer.h"
#include "CycleTest.h"
#include "Globals.h"

static const unsigned int scawCSMTimers[NUM_CSM_TIMERS] =
{
  /* CT_CSM_LOST_DELAY   */   1000,  //ms -delay before CSM CAN is assumed lost
};

static TICK_TIMER_S scastCSMTimers[NUM_CSM_TIMERS];

void ResetCSMTimer(CSM_TIMERS eCANTimer);

void initCSMCAN(void){
  unsigned int wIndex;
  for (wIndex = 0; wIndex < ((unsigned int)NUM_CSM_TIMERS); wIndex++){
   RegisterTimer(&scastCSMTimers[wIndex]);
   ResetCSMTimer((CSM_TIMERS)wIndex);
  }
}

void receiveCSMtemps(CANMessage message){
  if (message.id == CSM_TEMP_FIRST){
    CsmTemps[0] = (int16_t)((message.data[1]<<8)|(message.data[0]<<0));
    CsmTemps[1] = (int16_t)((message.data[3]<<8)|(message.data[2]<<0));
    CsmTemps[2] = (int16_t)((message.data[5]<<8)|(message.data[4]<<0));
    CsmTemps[3] = (int16_t)((message.data[7]<<8)|(message.data[6]<<0));
    return;
  }

/*  if (message.id == CSM_TEMP_SECOND){
    CsmTemps[4] = ((message.data[1]<<8)|(message.data[0]<<0));
    CsmTemps[5] = ((message.data[3]<<8)|(message.data[2]<<0));
    CsmTemps[6] = ((message.data[5]<<8)|(message.data[4]<<0));
    CsmTemps[7] = ((message.data[7]<<8)|(message.data[6]<<0));
    return;
  }*/
}

int CSMCANok(void){
  if (TimerExpired(&scastCSMTimers[CT_CSM_LOST_DELAY])){
    return 0;
  }
  else{
    return 1;
  }
}

void ResetCSMTimer(CSM_TIMERS eCANTimer)
{
  if (eCANTimer < NUM_CSM_TIMERS)
  {
    SetTimerWithMilliseconds(&scastCSMTimers[eCANTimer], scawCSMTimers[eCANTimer]);
  }
}
