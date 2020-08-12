#include "Network.h"
#include "Globals.h"
#include "TickTimer.h"

int sparkConnectTrig = 0;
int SparkNetOkPrev = 0;

static const unsigned int scawNetTimers[NUM_NET_TIMERS] =
{
  /* CT_NET_RETRY_DELAY   */   10000,  //ms -delay before trying to reconnect to the cloud
};

static TICK_TIMER_S scastNetTimers[NUM_NET_TIMERS];

void ResetNetTimer(NET_TIMERS eNetTimer);

void initNetwork(void){
  unsigned int wIndex;

  //Particle.connect();  //not needed unless SYSTEM_MODE is SEMI_AUTOMATIC or MANUAL

  for (wIndex = 0; wIndex < ((unsigned int)NUM_NET_TIMERS); wIndex++){
   RegisterTimer(&scastNetTimers[wIndex]);
   ResetNetTimer((NET_TIMERS)wIndex);
  }
}

void checkNetwork(void){
  if (Particle.connected()){
    //ResetNetTimer(CT_NET_RETRY_DELAY);
    SparkNetOk = 1;
  }
  else SparkNetOk = 0;

  ////not needed unless SYSTEM_MODE is SEMI_AUTOMATIC or MANUAL
  if (0){ //}(SparkNetOkPrev == 1) && (SparkNetOk == 0)){   //TimerExpired(&scastNetTimers[CT_NET_RETRY_DELAY])
    Particle.connect();
    ResetNetTimer(CT_NET_RETRY_DELAY);
    sparkConnectTrig = 0;
  }
  SparkNetOkPrev = SparkNetOk;
}

void ResetNetTimer(NET_TIMERS eNetTimer){
  if (eNetTimer < NUM_NET_TIMERS){
    SetTimerWithMilliseconds(&scastNetTimers[eNetTimer], scawNetTimers[eNetTimer]);
  }
}
