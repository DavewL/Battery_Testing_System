//Invntus CAN Protocol
#include "defines.h"
#include <carloop.h>

#define INVNTS_STATUS_ID 0x1

typedef enum
{
  CT_INVNTS_LOST_DELAY,
  NUM_INVNTS_TIMERS,
  FIRST_INVNTS_TIMER = 0
} INVNTS_TIMERS;

void initInvntsCAN(void);
void recInvntsStatus(CANMessage message);
int Invnts80AhCANok(void);
