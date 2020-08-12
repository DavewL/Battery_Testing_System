//#ifndef __REV2MESSAGES_H__
//#define __REV2MESSAGES_H__
#include <carloop.h>
#include "defines.h"

//constant declarations----------------------------------------------------
#define BMS_STATUS_ID 0x0C0
#define BATT_INFO_ID 0x0C1
#define BATT_CELL_VOLTS_0_ID 0x350
#define BATT_CELL_VOLTS_1_ID 0x351
#define BATT_MODULE_CURR_ID 0x46A
#define MODULE_SOC_ID 0x06A
#define MODULE_TEMPERATURE_ID 0x76A
#define MODULE_SN_FW 0x184

void receiveMessagesRev2(CANMessage);
//#endif
