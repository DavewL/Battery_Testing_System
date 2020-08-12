//cummins CAN Protocol
#include "defines.h"
#include <carloop.h>

#define CUMMINS_STATUS_ID 0x1

typedef enum
{
  CT_CUMMINS_LOST_DELAY,
  NUM_CUMMINS_TIMERS,
  FIRST_CUMMINS_TIMER = 0
} CUMMINS_TIMERS;

void initCumminsCAN(void);
void recCumminsStatus(CANMessage message);
int CumminsCANok(void);
