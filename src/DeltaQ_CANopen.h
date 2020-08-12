//DeltaQ_CANopen.h
#include <carloop.h>

#define DQ_REPLY_HB_ID 0x701
#define DQ_HB_ID 0x70A
#define DQ_RPDO1_ID 0x20A
#define DQ_IC1200_RPDO1_ID 0x20C
#define DQ_NMT_ID 0x000
#define DQ_SDO_ID 0x60A
#define DQ_TPDO1_ID 0x18A
#define DQ_TPDO2_ID 0x28A
#define DQ_IC1200_TPDO1_ID 0x18C


//0x20A
//0x30A
//0x40A
//0x50A

//typedef enum
//{
//  CT_DQCANOPEN_LOST_DELAY,
//  NUM_DQCAN_TIMERS,
//  FIRST_DQCAN_TIMER = 0
//} DQCAN_TIMERS1;


//RECEIVE MESSAGES
//void initDQCANopenRx(void);
void receiveMesDQCANopen(CANMessage);
//int DQCANopenok(void);

//TRANSMIT MESSAGES
void transmitDQGoOp(void);
void transmitRPDO1(void);
void transmitIC1200VoltsAmps(void);
void transmitNMT(void);
void transmitDQSDOready(void);
