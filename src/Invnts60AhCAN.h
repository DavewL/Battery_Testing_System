//Invntus CAN Protocol
#include "defines.h"
#include <carloop.h>

#define INVNTS_TPDO1_ID 0x1B1
#define INVNTS_TPDO2_ID 0x2B1
#define INVNTS_TPDO3_ID 0x3B1
#define INVNTS_TPDO4_ID 0x4B1
#define INVNTS_SDO_RESP_ID 0x5B1

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

#define INVNTS_VOLTS_SUBINDEX 0x09
#define INVNTS_CURRENT_SUBINDEX 0x0A
#define INVNTS_AVG_CURRENT_SUBINDEX 0x0B
#define INVNTS_CHRG_STATUS_SUBINDEX 0x16
#define INVNTS_SN_SUBINDEX 0x1C
#define INVNTS_HEATER_STATUS_SUBINDEX 0x92
#define INVNTS_MIN_CELL_TEMP 0xD8
#define INVNTS_MAX_CELL_TEMP 0xD9


typedef enum
{
  CT_INVNTS_60AH_LOST_DELAY,
  NUM_INVNTS_60AH_TIMERS,
  FIRST_INVNTS_60AH_TIMER = 0
} INVNTS_60AH_TIMERS;

void initInvnts60AhCAN(void);
void recInvnts60AhStatus(CANMessage message);
int Invnts60AhCANok(void);
void Invnts60AhSDOReadReq(int);
void Invnts60AhSDOWriteReq(int, int);
