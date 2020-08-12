#include "defines.h"
#include <carloop.h>

#define CSM_TEMP_FIRST 0x0602
#define CSM_TEMP_SECOND 0x0603

typedef enum
{
  CT_CSM_LOST_DELAY,
  NUM_CSM_TIMERS,
  FIRST_CSM_TIMER = 0
} CSM_TIMERS;

void initCSMCAN(void);
int CSMCANok(void);
void receiveCSMtemps(CANMessage message);
