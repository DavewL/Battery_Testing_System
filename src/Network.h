#include "application.h"

void initNetwork(void);
void checkNetwork(void);

typedef enum
{
  CT_NET_RETRY_DELAY,
  NUM_NET_TIMERS,
  FIRST_NET_TIMER = 0
} NET_TIMERS;
