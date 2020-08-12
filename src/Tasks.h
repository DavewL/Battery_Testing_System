//#ifndef __CANREC_H__
//#define __CANREC_H__

//#define FOUR_LINE_LCD 0
//#define THREEPOINTTWO_INCH_NEXTION 1

#define ONESECCOUNT 25
#define TWOSECCOUNT 50
#define FIVESECCOUNT 125
#define FIFTEENSECCOUNT 375

//#define BMS_LOG_MASK 0x0001
//#define TRAC_LOG_MASK 0x0002
//#define VCU_LOG_MASK 0x0004

//const int display = THREEPOINTTWO_INCH_NEXTION;


void TasksInit(void);
void Tasks5ms(void);
void Tasks10ms(void);
void Tasks20ms(void);
void Tasks40ms(void);
void Tasks80ms(void);
void Tasks160ms(void);
void Tasks320ms(void);
void Tasks640ms(void);
void Tasks1280ms(void);

//#endif
