#include <carloop.h>

#define SYS_STATUS_ID 0x0C0
#define CHARGE_INFO_ID 0x0D0
#define SYS_MEAS_ID 0x0E0
#define SYS_MEAS_TRACE_ID 0x0F0
#define CONTACTOR_STATES_ID 0x200
#define MODULE_INFO_ID 0x320
#define BMS_ID_INFO_ID 0x340
#define MODULE_ID_INDO_ID 0x380

typedef enum
{
  CT_CANR3_LOST_DELAY,
  NUM_VAL_BMSR3_TIMERS,
  FIRST_CANR3_TIMER = 0
} VAL_BMSR3_TIMERS;

void initValR3CAN(void);
void receiveMessagesRev3(CANMessage);
int BMSrev3CANok(void);
