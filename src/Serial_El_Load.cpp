#include "TickTimer.h"
#include "application.h"
#include "defines.h"
#include "Serial_El_Load.h"

void ResetElLoadTimer(ELLOAD_TIMERS eElLoadTimer);


static const unsigned int scawElLoadTimers[NUM_ELLOAD_TIMERS] =
{
  /* CT_200ms_DELAY   */   200,  //200 ms fixed delay
  /* CT_RESERVED      */   0,    //ms - RESERVED
};

static TICK_TIMER_S scastElLoadTimers[NUM_ELLOAD_TIMERS];


void initSerialElLoad(void){
  unsigned int wIndex;

  Serial1.begin(9600, SERIAL_8N1);

  for (wIndex = 0; wIndex < ((unsigned int)NUM_ELLOAD_TIMERS); wIndex++)
  {
    RegisterTimer(&scastElLoadTimers[wIndex]);
    ResetElLoadTimer((ELLOAD_TIMERS)wIndex);
  }

  Serial1.println("");
}

void putElLoadIntoRemote(void){
  Serial1.println("syst:rem");
  Serial1.println("*cls");
}

void turnElLoadOFF(void){
  Serial1.println("INP OFF");
}

void setElLoadToFixed(void){
  Serial1.println("FUNC:MODE FIX");
}

int startListRunning(void){
  static int stateVar = 0;
  int finished = 0;

  if (stateVar == 0){
    recallList();
    ResetElLoadTimer(CT_200ms_DELAY);
    stateVar++;
  }
  else if (stateVar == 1){
    if (TimerExpired(&scastElLoadTimers[CT_200ms_DELAY])){
      stateVar++;
    }
  }
  else if (stateVar == 2){
    setElLoadToList();
    turnElLoadON();
    ResetElLoadTimer(CT_200ms_DELAY);
    stateVar++;
  }
  else if (stateVar == 3){
    if (TimerExpired(&scastElLoadTimers[CT_200ms_DELAY])){
      stateVar++;
    }
  }
  else if (stateVar == 4){
    triggerElLoad();
    finished = 1;
    stateVar = 0;
  }
  return finished;

}

void recallList(void){
  Serial1.println("LIST:RCL 2");
}

void setElLoadToList(void){
  Serial1.println("FUNC:MODE LIST");
}

void turnElLoadON(void){
  Serial1.println("INP ON");
}

void triggerElLoad(void){
  Serial1.println("*TRG");
}

void setElLoad2PwrMode(void){
  //send command to set to power mode
  Serial1.println("FUNC POW");
  turnElLoadON();
}

void sendFixPwrLevel(int powerLevel){
  //send command to set the power level
  //String tempStringPow = "POW ";
  //tring tempPowLevel = String::format("%d")
  //tempStringPow.append()
  Serial1.printf("POW %d\n",powerLevel);
  //Serial.printf("POW %d\n",powerLevel);
}

void ResetElLoadTimer(ELLOAD_TIMERS eElLoadTimer)
{
  if (eElLoadTimer < NUM_ELLOAD_TIMERS)
  {
    SetTimerWithMilliseconds(&scastElLoadTimers[eElLoadTimer], scawElLoadTimers[eElLoadTimer]);
  }
}
