#include "TickTimer.h"
#include "application.h"
//#include "timers.h"

#define TICK_TIMER_EXPIRED_BIT ((unsigned int) 0x80000000)
#define TICK_TIMER_EXPIRED_MASK ((unsigned int)~(TICK_TIMER_EXPIRED_BIT))

TICK_TIMER_S * scpstHeadTimer;  /*head of timer list*/

void InitTickTimers(void)
{
  scpstHeadTimer = NULL;
}

void RegisterTimer(TICK_TIMER_S * pstTimer)
{
  TICK_TIMER_S * pstCurrentTimer;
  if (scpstHeadTimer == NULL)
  {
    scpstHeadTimer = pstTimer;
    pstTimer->pstNextTimer = NULL;
  }
  else
  {
    for (pstCurrentTimer = scpstHeadTimer;
          pstCurrentTimer->pstNextTimer != NULL;
          pstCurrentTimer = pstCurrentTimer->pstNextTimer)
    {
      if (pstCurrentTimer->pstNextTimer == pstTimer)
      {
        return;
      }
    }
    pstCurrentTimer->pstNextTimer = pstTimer;
    pstTimer->pstNextTimer = NULL;
  }
  ClearTimer(pstTimer);
}

void UpdateTimers(void)
{
  TICK_TIMER_S * pstCurrentTimer;
  for (pstCurrentTimer = scpstHeadTimer;
      pstCurrentTimer != NULL;
      pstCurrentTimer = pstCurrentTimer->pstNextTimer)
  {
    switch (pstCurrentTimer->wExpirationTime)
    {
      case TICK_TIMER_EXPIRED_BIT:
      case TICK_TIMER_EXPIRED_MASK:
        break;
      case 0:
        pstCurrentTimer->wExpirationTime = TICK_TIMER_EXPIRED_BIT;
        break;
      default:
        pstCurrentTimer->wExpirationTime--;
        break;
    }
  }
}

void SetTimerWithMilliseconds(TICK_TIMER_S * pstTimer, unsigned int wMilliseconds)
{
  if ((wMilliseconds / 5) < TICK_TIMER_EXPIRED_MASK)
  {
    pstTimer->wExpirationTime = wMilliseconds / 5; //5 ms refresh interval
  }
  else
  {
    pstTimer->wExpirationTime = TICK_TIMER_EXPIRED_MASK -1;
  }
}

void ClearTimer(TICK_TIMER_S * kpstTimer)
{
  kpstTimer->wExpirationTime = TICK_TIMER_EXPIRED_MASK;
}

bool TimerRunning(const TICK_TIMER_S * kpstTimer)
{
  return (kpstTimer->wExpirationTime < TICK_TIMER_EXPIRED_MASK);
}

bool TimerExpired(const TICK_TIMER_S * kpstTimer)
{
  return (kpstTimer->wExpirationTime & TICK_TIMER_EXPIRED_BIT) != 0;
}

unsigned int wMsTimerWillExpireIn(const TICK_TIMER_S * kpstTimer)
{
  //if (kpstTimer->wExpirationTime & TICK_TIMER_EXPIRED_MASK) * 5 >0xFFFFFFFF)
  //{
  //  return 0xFFFFFFFF;
  //}
  //else
  //{
    return ((kpstTimer->wExpirationTime & TICK_TIMER_EXPIRED_MASK) * 5);
  //}
}
