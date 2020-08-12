
void initSerialElLoad(void);
void putElLoadIntoRemote(void);
void setElLoadToFixed(void);
void turnElLoadOFF(void);
void recallList(void);
void setElLoadToList(void);
int startListRunning(void);
void turnElLoadON(void);
void triggerElLoad(void);

void setElLoad2PwrMode(void);

void sendFixPwrLevel(int powerLevel);

typedef enum
{
  CT_200ms_DELAY,       //200 ms fixed delay
  CT_RESERVED,          //ms - RESERVED
  NUM_ELLOAD_TIMERS,
  FIRST_ELLOAD_TIMER = 0
} ELLOAD_TIMERS;
