#include "application.h"

#define BMS_LOG_MASK 0x0001
#define TRAC_LOG_MASK 0x0002
#define VCU_LOG_MASK 0x0004
#define CSM_TEMP_LOG_MASK 0x0008
#define TEST_SUPERVISOR_LOG_MASK 0x0010

typedef struct {
  int Seconds;
  float Current;
  float Volts;
  float Power;
} ElLoadFrame;

typedef enum
{
  CT_SD_NEXT_LINE_DELAY,
  CT_SD_CLOSE_DELAY,
  CT_SD_OPEN_DELAY,
  NUM_SD_TIMERS,
  FIRST_SD_TIMER = 0
} SD_TIMERS;

 void initSD(int what2log);
 void initSDcard(void);
 void openReadWriteFiles(int what2log);

 //write to SD card
 //void LogBMS2SD(void);
 //void LogTRAC2SD(void);
 //void LogVCU2SD(void);
 //void LogCsmTemp2SD(void);
 //void LogTestSuper2SD(void);
 void LogUserString(String logString);
 void Log2SD(int what2log);

 int ReadFrameFromFile(void);
 void ReadFrameLine(void);
 void RestartFrameFileLines(void);
