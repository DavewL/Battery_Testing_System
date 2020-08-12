//Invntus CAN Protocol
#include "defines.h"
#include <carloop.h>

#define INVNTS_TPDO3_ID 0x3B1
#define INVNTS_CELL_VOLTS_ID 0x3B2
#define INVNTS_MUX_1 0x01
#define INVNTS_MUX_2 0x02
#define INVNTS_MUX_3 0x03

#define CELL_1_MUX 0x32
#define CELL_2_MUX 0x33
#define CELL_3_MUX 0x34
#define CELL_4_MUX 0x35
#define CELL_5_MUX 0x36
#define CELL_6_MUX 0x37
#define CELL_7_MUX 0x38
#define CELL_8_MUX 0x39

typedef enum
{
  CT_INVNTS_LOST_DELAY,
  NUM_INVNTS_TIMERS,
  FIRST_INVNTS_TIMER = 0
} INVNTS_TIMERS;

void initInvntsCAN(void);
void recInvntsStatus(CANMessage message);
int Invnts80AhCANok(void);
int InvntsReqModVolts(void);
int InvntsReqModCurrent(void);
int InvntsReqHeaterStatus(void);
