typedef struct TICK_TIMER_S_TAG
{
  unsigned int wExpirationTime;
  struct TICK_TIMER_S_TAG * pstNextTimer;
} TICK_TIMER_S;

void InitTickTimers(void);
void RegisterTimer(TICK_TIMER_S * pstTimer);
void UpdateTimers(void);
void SetTimerWithMilliseconds(TICK_TIMER_S * pstTimer, unsigned int wTicks);
void ClearTimer(TICK_TIMER_S * kpstTimer);
bool TimerRunning(const TICK_TIMER_S * kpstTimer);
bool TimerExpired(const TICK_TIMER_S * kpstTimer);
unsigned int wMsTimerWillExpireIn(const TICK_TIMER_S * kpstTimer);
