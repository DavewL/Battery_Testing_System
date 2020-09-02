#include "Log2SdCard.h"
#include "CANrec.h"
#include <SdFat.h>
#include "defines.h"
#include "Globals.h"
#include "Rev3Messages.h"
#include "CumminsCAN.h"
#include "CycleTest.h"
#include "TickTimer.h"
#include "Serial_El_Load.h"
#include "Display.h"
#include "DeltaQ_CANopen.h"
#include "Invnts80AhCAN.h"


//SD file declarations-----------------------------------
SdFat sd;
const uint8_t chipSelect = SS;
SdFile myFile;
SdFile readFramesFile;
SdFile readConfigTxtFile;
int SDcardInitOK = 0;
String strFileName;
int frameDelaySec = 0;
int frameDelaymSec = 100;
String strReadFileName;
int attemptedOpen = 0;
int frameNumber = 0;
String logFileHeader = "";

extern String testState2String;
extern String heaterState2String;

ElLoadFrame NextLoadFrame;
ElLoadFrame CurrLoadFrame;

int BMSlogging;
int TRAClogging;
int VCUlogging;
int CSMlogging;
int TEST_SPRVSRlogging;
unsigned int BMS_t_prev = 0;
unsigned int VCU_t_prev = 0;
unsigned int TRAC_t_prev = 0;
unsigned int CSM_t_prev = 0;
unsigned int SUPER_t_prev = 0;

static const unsigned int scawSDtimers[NUM_SD_TIMERS] =
{
  /* CT_SD_NEXT_LINE_DELAY   */   1000,  //ms -delay before reading the next line --gets updated after each read.
  /* CT_SD_CLOSE_DELAY       */   60,  //ms -delay after closing file to attempt reopen
  /* CT_SD_OPEN_DELAY        */   1000,  //ms -delay before attempt to reopen file

};

static TICK_TIMER_S scastSDtimers[NUM_SD_TIMERS];

void ResetSDTimer(SD_TIMERS eSDTimer);
int openReadFrameFile(String fileName);
void closeReadFrameFile(void);

//void initSD(int);
//void LogBMS2SD(void);

int openReadFrameFile(String fileName){
  //Serial.println("attempting to open READ file");
  //Serial.println(fileName);
  int tempReadStat = readFramesFile.open(fileName, O_READ);
  //Serial.println(tempReadStat);
  if (tempReadStat){
    //Serial.print(fileName);
    //Serial.println(" OPENED");
    char fileLineChars[400];
    readFramesFile.fgets(fileLineChars,400); //read and discard first line
    //Serial.println(fileLineChars);
    //readFramesFile.fgets(fileLineChars,400); //read and discard second line
    //Serial.println(fileLineChars);
    return 1;
  }
  else return 0;
}

void closeReadFrameFile(void){
  //Serial.println("attempting to close READ file");
  readFramesFile.close();
  //readFrameFileExists = 0;
}

void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(Time.year(), Time.month(), Time.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(Time.hour(), Time.minute(), Time.second());
}//------------------------------------------------------------------------------

void initSDcard(void){
  SdFile::dateTimeCallback(dateTime); //make files on SD card have correct creation date/time
  String fieldString;
  char fileLineChars[200];
  String fileLine;
  String tempUserString;
  bool eofFound = false;
  int j = 5;
  int n = 0;  //number of characters read
  int i = 0;
  int tempLogItems = 0;
  unsigned int wIndex;
  String configTxtFileName = "CONFIG.txt";
  String fieldSubStringHead;
  String fieldSubStringValue;

  for (wIndex = 0; wIndex < ((unsigned int)NUM_SD_TIMERS); wIndex++){
   RegisterTimer(&scastSDtimers[wIndex]);
   ResetSDTimer((SD_TIMERS)wIndex);
  }

  SDcardInitOK = sd.begin(chipSelect, SD_SCK_HZ(16 * MHZ));  //SD_SCK_HZ(2 * MHZ)  //SPI_FULL_SPEED
  //Serial.println("opening SD");
  if (SDcardInitOK){
    int tempReadStat = readConfigTxtFile.open(configTxtFileName, O_READ);
    while ((tempReadStat)&&(!eofFound)){
      n = readConfigTxtFile.fgets(fileLineChars,200);
      fieldString = fileLineChars;
      if (n > 0){
        if (fieldString.substring(0,1) == "#"){
          //---------search for the field name--------------
          while((fieldString.substring(i,i+1) != "=") && (i < n-1)){
            i++;
          }
          if (i < (n-1)){
            fieldSubStringHead = fieldString.substring(0,i);
          }
          //Serial.println(fieldSubStringHead);

          //----------search for the field value-------------
          i++;
          j = i;
          while((fieldString.substring(i,i+1) != ";")&&(i < n-1)){
            i++;
          }
          if (1){  //i < n
            fieldSubStringValue = fieldString.substring(j,(n-2));
          }
          //Serial.println(fieldSubStringValue);

          //-------------configure settings based on config file values--------
          if (fieldSubStringHead == "#BATT_TYPE"){
            if (fieldSubStringValue == "CUMMINS_R1"){
                battType = CUMMINS_REV1;
              }
              else if (fieldSubStringValue == "VALENCE_R3"){
                battType = VALENCE_REV3;
              }
              else if (fieldSubStringValue == "INVNTS_80AH"){
                battType = INVNTS_80AH;
              }
              else{
                battType = UNKNOWN_BATT;
              }
          }
          else if (fieldSubStringHead == "#FUNCTION"){
            if (fieldSubStringValue == "CYCLE"){
              testType = CYCLE;
            }
            else if (fieldSubStringValue == "VEHICLE_LOG"){
              testType = VEHICLE_LOG;
            }
            else{
              testType = CYCLE;
            }
          }
          else if (fieldSubStringHead == "#MAX_CHARGE_PERCNT"){
            maxChargePercent = fieldSubStringValue.toInt();
          }
          else if (fieldSubStringHead == "#MIN_DISCHARGE_PERCNT"){
            minDischargePercent = fieldSubStringValue.toInt();
          }
          else if (fieldSubStringHead == "#DISCHARGER_CYCLE_TYPE"){
            if (fieldSubStringValue == "SD_REPLAY"){
              dischargerType = SD_REPLAY;
            }
            else if (fieldSubStringValue == "LIST"){
              dischargerType = LIST;
            }
            else{
              dischargerType = LIST;
            }
          }
          else if (fieldSubStringHead == "#SD_LOG_RATE"){
            sdLogRate = fieldSubStringValue.toInt();
          }
          else if (fieldSubStringHead == "#SD_LOG_ITEMS"){
            if (fieldSubStringValue == "BMS_LOG_MASK"){
              tempLogItems = tempLogItems|BMS_LOG_MASK;
            }
            else if (fieldSubStringValue == "TRAC_LOG_MASK"){
              tempLogItems = tempLogItems|TRAC_LOG_MASK;
            }
            else if (fieldSubStringValue == "VCU_LOG_MASK"){
              tempLogItems = tempLogItems|VCU_LOG_MASK;
            }
            else if (fieldSubStringValue == "CSM_TEMP_LOG_MASK"){
              tempLogItems = tempLogItems|CSM_TEMP_LOG_MASK;
            }
            else if (fieldSubStringValue == "TEST_SUPERVISOR_LOG_MASK"){
              tempLogItems = tempLogItems|TEST_SUPERVISOR_LOG_MASK;
            }
            what2logTxt = tempLogItems;
          }
          else if (fieldSubStringHead == "#LOGFILE_HEADER"){
            logFileHeader = fieldSubStringValue;
          }
        }
      }
      else {
        eofFound = true;
        readConfigTxtFile.close();
      }
      i = 0;
    }
  }
  //Serial.print(fieldSubStringValue);
}

void openReadWriteFiles(int what2log){
  String strFileName2;
  String tempBattType;
  BMSlogging = (what2log & BMS_LOG_MASK);     //log BMS data
  TRAClogging = (what2log & TRAC_LOG_MASK)>>1;    //log traction system data
  VCUlogging = (what2log & VCU_LOG_MASK)>>2;     //log general vehicle data
  CSMlogging = (what2log & CSM_TEMP_LOG_MASK)>>3; //log csm temperature data
  TEST_SPRVSRlogging = (what2log & TEST_SUPERVISOR_LOG_MASK)>>4;
  unsigned int nowTime = Time.now();

  strReadFileName = "EL_LOAD_FRAMES.csv";

  if (battType == VALENCE_REV3){
    tempBattType = "VALR3";
  }
  else if (battType == CUMMINS_REV1){
    tempBattType = "BRAMR1";
  }
  else if (battType == INVNTS_80AH){
    tempBattType = "INVNTS80AH";
  }
  strFileName = String::format("%d_BATT_SN_",Time.now());
  Serial.println(strFileName);
  strFileName.concat(tempBattType);
  strFileName.concat("_");
  strFileName2 = String::format("%d",battSN);
  strFileName.concat(strFileName2);
  strFileName.concat(".csv");
  Serial.println(strFileName);

/////-------------------------------------------------------------------INIT LOG FILE------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if (SDcardInitOK){
    BMS_t_prev = nowTime;
    VCU_t_prev = nowTime;
    TRAC_t_prev = nowTime;
    CSM_t_prev = nowTime;
    SUPER_t_prev = nowTime;
      if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){                                                                                                                                                                                                  //--
          // if the file opened okay, write to it:                                                                                                                                                                                                                 //--
          myFile.println("LOG FORMAT V2");
          if (logFileHeader != ""){
            myFile.println(logFileHeader);                                                                                                                                                                                                                         //--
          }
          if (BMSlogging == 1){                                                                                                                                                                                                                                    //--
              myFile.println("#BMS,TIME,INTERVAL,BATTERY TYPE,FAULT STATUS,VOLTAGE,CURRENT,MIN CELL TEMP,MAX CELL TEMP,SOC,CELL1,CELL2,CELL3,CELL4,CELL5,CELL6,CELL7,CELL8,AMPHOURS,CUML AMPHOURS,WATTHOURS,CUML WATTHOURS,MAX DISCH CURR,MAX REGEN CURR,BATT MODULE SN,BMS CHRG CURR SETPNT,BMS CHRG VOLT SETPNT,MODULE SOH");  //--
          }                                                                                                                                                                                                                                                        //--
          if (TRAClogging == 1){                                                                                                                                                                                                                                   //--
             myFile.println("#TRACTION,TIME,INTERVAL,LEFT FAULT STATUS,LEFT VOLTAGE,LEFT MOTOR SPEED,LEFT RMS CURRENT,LEFT MOTOR TEMP,LEFT CONTRLR TEMP,RIGHT FAULT STATUS,RIGHT VOLTAGE,RIGHT MOTOR SPEED,RIGHT RMS CURRENT,RIGHT MOTOR TEMP,RIGHT CONTRLR TEMP");         //--
          }                                                                                                                                                                                                                                                        //--
          if (VCUlogging == 1){                                                                                                                                                                                                                                    //--
              myFile.println("#VCU,TIME,INTERVAL,FAULT STATUS,JOYSTICK DRIVE,JOYSTICK STEER,JOYSTICK LIFT,STEER ANGLE,STEER CURRENT,STEER TEMPERATURE,LIFT HEIGHT,LIFT CURRENT,LIFT TEMPERATURE");                                                                          //--
          }                                                                                                                                                                                                                                                        //--
          if (CSMlogging == 1){                                                                                                                                                                                                                                    //--
              myFile.println("#CSM_TEMP,TIME,INTERVAL,TEMP_1,TEMP_2,TEMP_3,TEMP_4,TEMP_5,TEMP_6,TEMP_7,TEMP_8");                                                                                                                                                            //--
          }                                                                                                                                                                                                                                                        //--
          if (TEST_SPRVSRlogging == 1){                                                                                                                                                                                                                            //--
            myFile.println("#TEST_SUPERVISOR,TIME,INTERVAL,TEST STATEMACHINE,HEATER STATEMACHINE,DELTA-Q VOLTS,DELTA-Q CURRENT,HEATER RELAY,KEY RELAY,CHARGE RELAY,CHRG CURR SETPNT,CHRG VOLT SETPNT");                                                                                                       //--
          }                                                                                                                                                                                                                                                        //--
          // close the file:                                                                                                                                                                                                                                       //--
          myFile.close();
          myFileOk = 1;                                                                                                                                                                                                                                            //--
      }
      else myFileOk = 0;
/////--------------------------------INIT READ FILE--------------------------------
      readFrameFileExists = openReadFrameFile(strReadFileName);                                                                                                                                                                                                    //--
  }                                                                                                                                                                                                                                                                //--

}

/*void initSD(int what2log){
    SdFile::dateTimeCallback(dateTime); //make files on SD card have correct creation date/time

    BMSlogging = (what2log & BMS_LOG_MASK);     //log BMS data
    TRAClogging = (what2log & TRAC_LOG_MASK)>>1;    //log traction system data
    VCUlogging = (what2log & VCU_LOG_MASK)>>2;     //log general vehicle data
    CSMlogging = (what2log & CSM_TEMP_LOG_MASK)>>3; //log csm temperature data
    TEST_SPRVSRlogging = (what2log & TEST_SUPERVISOR_LOG_MASK)>>4;
//    String strReadFileName;
    unsigned int wIndex;

    for (wIndex = 0; wIndex < ((unsigned int)NUM_SD_TIMERS); wIndex++){
     RegisterTimer(&scastSDtimers[wIndex]);
     ResetSDTimer((SD_TIMERS)wIndex);
    }

    SDcardInitOK = sd.begin(chipSelect, SPI_HALF_SPEED);
    String tempBattType;
    String strFileName2;

    String strBattTypeFile = "VALENCE_R3_BATT.txt";
    int tempReadStat = readConfigTxtFile.open(strBattTypeFile, O_READ);
    if (tempReadStat){
      battType = VALENCE_REV3;
    }
    strBattTypeFile = "CUMMINS_R1_BATT.txt";
    tempReadStat = readConfigTxtFile.open(strBattTypeFile, O_READ);
    if (tempReadStat){
      battType = CUMMINS_REV1;
    }

    if (battType == VALENCE_REV3){
      tempBattType = "VALR3";
    }
    else if (battType == CUMMINS_REV1){
      tempBattType = "BRAMR1";
    }
    strFileName = String::format("%d_BATT_SN_",Time.now());
    strFileName.concat(tempBattType);
    strFileName.concat("_");
    strFileName2 = String::format("%d",battSN);
    strFileName.concat(strFileName2);
    strFileName.concat(".csv");
    strReadFileName = "EL_LOAD_FRAMES.csv";

/////-------------------------------------------------------------------INIT LOG FILE------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if (SDcardInitOK){
        if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){                                                                                                                                                                                                  //--
            // if the file opened okay, write to it:                                                                                                                                                                                                                 //--
            myFile.println("LOG FORMAT V2");                                                                                                                                                                                                                         //--
            if (BMSlogging == 1){                                                                                                                                                                                                                                    //--
                myFile.println("#BMS,TIME,BATTERY TYPE,FAULT STATUS,VOLTAGE,CURRENT,MIN CELL TEMP,MAX CELL TEMP,SOC,CELL1,CELL2,CELL3,CELL4,CELL5,CELL6,CELL7,CELL8,AMPHOURS,CUML AMPHOURS,WATTHOURS,CUML WATTHOURS,MAX DISCH CURR,MAX REGEN CURR,BATT MODULE SN");  //--
            }                                                                                                                                                                                                                                                        //--
            if (TRAClogging == 1){                                                                                                                                                                                                                                   //--
               myFile.println("#TRACTION,TIME,LEFT FAULT STATUS,LEFT VOLTAGE,LEFT MOTOR SPEED,LEFT RMS CURRENT,LEFT MOTOR TEMP,LEFT CONTRLR TEMP,RIGHT FAULT STATUS,RIGHT VOLTAGE,RIGHT MOTOR SPEED,RIGHT RMS CURRENT,RIGHT MOTOR TEMP,RIGHT CONTRLR TEMP");         //--
            }                                                                                                                                                                                                                                                        //--
            if (VCUlogging == 1){                                                                                                                                                                                                                                    //--
                myFile.println("#VCU,TIME,FAULT STATUS,JOYSTICK DRIVE,JOYSTICK STEER,JOYSTICK LIFT,STEER ANGLE,STEER CURRENT,STEER TEMPERATURE,LIFT HEIGHT,LIFT CURRENT,LIFT TEMPERATURE");                                                                          //--
            }                                                                                                                                                                                                                                                        //--
            if (CSMlogging == 1){                                                                                                                                                                                                                                    //--
                myFile.println("#CSM_TEMP,TIME,TEMP_1,TEMP_2,TEMP_3,TEMP_4,TEMP_5,TEMP_6,TEMP_7,TEMP_8");                                                                                                                                                            //--
            }                                                                                                                                                                                                                                                        //--
            if (TEST_SPRVSRlogging == 1){                                                                                                                                                                                                                            //--
              myFile.println("#TEST_SUPERVISOR,TIME,TEST STATEMACHINE,HEATER STATEMACHINE,DELTA-Q VOLTS,DELTA-Q CURRENT,HEATER RELAY,KEY RELAY,CHARGE RELAY");                                                                                                       //--
            }                                                                                                                                                                                                                                                        //--
            // close the file:                                                                                                                                                                                                                                       //--
            myFile.close();
            myFileOk = 1;                                                                                                                                                                                                                                            //--
        }
        else myFileOk = 0;
/////--------------------------------INIT READ FILE--------------------------------
        readFrameFileExists = openReadFrameFile(strReadFileName);                                                                                                                                                                                                    //--
    }                                                                                                                                                                                                                                                                //--
/////--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
}*/

/*void LogBMS2SD(){
    float fCombinedModuleVolts = (float)(battCell1mv+battCell2mv+battCell3mv+battCell4mv+battCell5mv+battCell6mv+battCell7mv+battCell8mv)/1000;
    if (SDcardInitOK && !flagSDPause && (battCell1mv != 111)){
      if (battType == VALENCE_REV3){
        if (BMSrev3CANok()){
          if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
              cumlAmpHrs = cumlAmpHrs + ampHours;
              cumlWattHrs = cumlWattHrs + wattHours;
              // if the file opened okay, write to it:
              myFile.printf("#BMS,%d,VALENCE,%d,%.3f,%.1f,%d,%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%d\n"
              ,Time.now(),BMSstatus,fCombinedModuleVolts,battCurrent,moduleMinTemperature,moduleMaxTemperature,moduleSOCscale,battCell1mv,battCell2mv,battCell3mv,battCell4mv,battCell5mv,battCell6mv,battCell7mv,battCell8mv,ampHours,cumlAmpHrs,wattHours,cumlWattHrs,maxDischargeCurrent,maxRegenCurrent,battSN);
              // close the file:
              myFile.close();
              ampHours = 0;
              wattHours = 0;
          }
        }
      }
      else if (battType == CUMMINS_REV1){
        if (CumminsCANok()){
          if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
              cumlAmpHrs = cumlAmpHrs + ampHours;
              cumlWattHrs = cumlWattHrs + wattHours;
              // if the file opened okay, write to it:
              myFile.printf("#BMS,%d,CUMMINS,%d,%.3f,%.1f,%d,%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%d\n"
              ,Time.now(),BMSstatus,fCombinedModuleVolts,battCurrent,moduleMinTemperature,moduleMaxTemperature,moduleSOCscale,battCell1mv,battCell2mv,battCell3mv,battCell4mv,battCell5mv,battCell6mv,battCell7mv,battCell8mv,ampHours,cumlAmpHrs,wattHours,cumlWattHrs,maxDischargeCurrent,maxRegenCurrent,battSN);
              // close the file:
              myFile.close();
              ampHours = 0;
              wattHours = 0;
          }
        }
      }
    }
}*/

/*void LogTRAC2SD(){
    if (SDcardInitOK && !flagSDPause){
        if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
            // if the file opened okay, write to it:
            //myFile.printf("#BMS,%d,%d,%.3f,%d,%.3f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f\n",Time.now(),FAULT STATUS,fCombinedModuleVolts,battCurrent,tempTemp,tempSOC,battCell1mv,battCell2mv,battCell3mv,battCell4mv,battCell5mv,battCell6mv,battCell7mv,battCell8mv);
            //myFile.printf("#TRACTION,%.1f\n",moduleSOCscale);
            // close the file:
            myFile.close();
        }
    }
}*/

/*void LogVCU2SD(){
    if (SDcardInitOK && !flagSDPause){
        if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
            // if the file opened okay, write to it:
            //myFile.printf("#BMS,%d,%d,%.3f,%d,%.3f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f\n",Time.now(),FAULT STATUS,fCombinedModuleVolts,battCurrent,tempTemp,tempSOC,battCell1mv,battCell2mv,battCell3mv,battCell4mv,battCell5mv,battCell6mv,battCell7mv,battCell8mv);
            //myFile.printf("#VCU,%.1f\n",moduleSOCscale);
            // close the file:
            myFile.close();
        }
    }
}*/

void LogUserString(String logString)
{
  String tempString = "#USER_STRING,";
  if (SDcardInitOK){
      if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
          // if the file opened okay, write to it:
          tempString.concat(logString);
          myFile.println(tempString);
          // close the file:
          myFile.close();
      }
  }
}

/*void LogCsmTemp2SD(void){
  if (SDcardInitOK && !flagSDPause && BMSrev3CANok()){
    float tempCSM1 = (float)CsmTemps[0]/10;
    float tempCSM2 = (float)CsmTemps[1]/10;
    float tempCSM3 = (float)CsmTemps[2]/10;
    float tempCSM4 = (float)CsmTemps[3]/10;
    float tempCSM5 = 500.0;
    float tempCSM6 = 600.0;
    float tempCSM7 = 700.0;
    float tempCSM8 = 800.0;
    if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
        // if the file opened okay, write to it:
        myFile.printf("#CSM_TEMP,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",Time.now(),tempCSM1,tempCSM2,tempCSM3,tempCSM4,tempCSM5,tempCSM6,tempCSM7,tempCSM8);
        //myFile.printf("#VCU,%.1f\n",moduleSOCscale);
        // close the file:
        myFile.close();
    }
  }
}*/

/*void LogTestSuper2SD(void){
  if (SDcardInitOK && !flagSDPause){
      if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
          // if the file opened okay, write to it:
                                      //TIME,TEST STATEMACHINE,HEATER STATEMACHINE,DELTA-Q VOLTS,DELTA-Q CURRENT,HEATER RELAY,KEY RELAY,CHARGE RELAY"
          myFile.printf("#TEST_SUPERVISOR,%d,%s,%s,%.3f,%.3f,%d,%d,%d\n",Time.now(),testState2String,heaterState2String,DQcurrent,DQvoltage,heaterStatus,keySignalStatus,rechargeState);
          //myFile.printf("#VCU,%.1f\n",moduleSOCscale);
          // close the file:
          myFile.close();
      }
  }
}*/

void Log2SD(int what2log){
  float fCombinedModuleVolts = 0;
  String tempStringSD = "";
  String tempStringSD2 = "";
  unsigned int interval = 0;

  if (SDcardInitOK){
    if (myFile.open(strFileName, O_RDWR | O_CREAT | O_APPEND)){
      // if the file opened okay, write to it:
      int BMSlogging = (what2log & BMS_LOG_MASK);     //log BMS data
      int TRAClogging = (what2log & TRAC_LOG_MASK)>>1;    //log traction system data
      int VCUlogging = (what2log & VCU_LOG_MASK)>>2;     //log general vehicle data
      int CSMlogging = (what2log & CSM_TEMP_LOG_MASK)>>3; //log csm temperature data
      int TEST_SPRVSRlogging = (what2log & TEST_SUPERVISOR_LOG_MASK)>>4;

      if (BMSlogging == 1){
        interval = Time.now()-BMS_t_prev;
        if (battType == VALENCE_REV3){
          if (BMSrev3CANok()){
            fCombinedModuleVolts = (float)(battCell1mv+battCell2mv+battCell3mv+battCell4mv+battCell5mv+battCell6mv+battCell7mv+battCell8mv)/1000;
            cumlAmpHrs = cumlAmpHrs + ampHours;
            cumlWattHrs = cumlWattHrs + wattHours;
            // if the file opened okay, write to it:
            myFile.printf("#BMS,%d,%d,VALENCE,%08x,%.3f,%.1f,%d,%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%d,%.1f,%.1f,NA\n"
            ,Time.now(),interval,BMSstatusWord,fCombinedModuleVolts,battCurrent,moduleMinTemperature,moduleMaxTemperature,moduleSOCscale,battCell1mv,battCell2mv,battCell3mv,battCell4mv,battCell5mv,battCell6mv,battCell7mv,battCell8mv,ampHours,cumlAmpHrs,wattHours,cumlWattHrs,maxDischargeCurrent,maxRegenCurrent,battSN,BMSchargeCurrSetpoint,BMSchargeVoltSetpoint);
            ampHours = 0;
            wattHours = 0;
            BMS_t_prev = Time.now();
          }
        }
        else if (battType == CUMMINS_REV1){
          if (CumminsCANok()){
            fCombinedModuleVolts = battVoltage;
            cumlAmpHrs = cumlAmpHrs + ampHours;
            cumlWattHrs = cumlWattHrs + wattHours;
            // if the file opened okay, write to it:
            myFile.printf("#BMS,%d,%d,CUMMINS,%d,%.3f,%.1f,%d,%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%d,%.1f,%.1f,NA\n"
            ,Time.now(),interval,BMSstatus,fCombinedModuleVolts,battCurrent,moduleMinTemperature,moduleMaxTemperature,moduleSOCscale,battCell1mv,battCell2mv,battCell3mv,battCell4mv,battCell5mv,battCell6mv,battCell7mv,battCell8mv,ampHours,cumlAmpHrs,wattHours,cumlWattHrs,maxDischargeCurrent,maxRegenCurrent,battSN,BMSchargeCurrSetpoint,BMSchargeVoltSetpoint);
            ampHours = 0;
            wattHours = 0;
            BMS_t_prev = Time.now();
          }
        }
        else if (battType == INVNTS_80AH){
          if (Invnts80AhCANok()){
            fCombinedModuleVolts = battVoltage;
            cumlAmpHrs = cumlAmpHrs + ampHours;
            cumlWattHrs = cumlWattHrs + wattHours;
            // if the file opened okay, write to it:
            myFile.printf("#BMS,%d,%d,INVNTS_80AH,%d,%.3f,%.1f,%d,%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.1f,%.1f,%d,%.1f,%.1f,%d\n"
            ,Time.now(),interval,BMSstatus,fCombinedModuleVolts,battCurrent,moduleMinTemperature,moduleMaxTemperature,moduleSOCscale,battCell1mv,battCell2mv,battCell3mv,battCell4mv,battCell5mv,battCell6mv,battCell7mv,battCell8mv,ampHours,cumlAmpHrs,wattHours,cumlWattHrs,maxDischargeCurrent,maxRegenCurrent,battSN,BMSchargeCurrSetpoint,BMSchargeVoltSetpoint,InvntsSOH);
            ampHours = 0;
            wattHours = 0;
            BMS_t_prev = Time.now();
          }
        }
      }
      if (TRAClogging == 1){
        interval = Time.now()-TRAC_t_prev;
        TRAC_t_prev = Time.now();
      }
      if (VCUlogging == 1){
        interval = Time.now()-VCU_t_prev;
        VCU_t_prev = Time.now();
      }
      if (CSMlogging == 1){
        float tempCSM1 = (float)CsmTemps[0]/10;
        float tempCSM2 = (float)CsmTemps[1]/10;
        float tempCSM3 = (float)CsmTemps[2]/10;
        float tempCSM4 = (float)CsmTemps[3]/10;
        float tempCSM5 = 500.0;
        float tempCSM6 = 600.0;
        float tempCSM7 = 700.0;
        float tempCSM8 = 800.0;
        interval = Time.now() - CSM_t_prev;
        myFile.printf("#CSM_TEMP,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",Time.now(),interval,tempCSM1,tempCSM2,tempCSM3,tempCSM4,tempCSM5,tempCSM6,tempCSM7,tempCSM8);
        CSM_t_prev = Time.now();
      }
      if (TEST_SPRVSRlogging == 1){
        interval = Time.now() - SUPER_t_prev;
        tempStringSD = String::format("#TEST_SUPERVISOR,%d,%d,",Time.now(),interval);
        tempStringSD.concat(testState2String);
        tempStringSD.concat(",");
        tempStringSD.concat(heaterState2String);
        tempStringSD2 = String::format(",%.3f,%.3f,%d,%d,%d,%.1f,%.1f\n",DQvoltage,DQcurrent,heaterStatus,keySignalStatus,rechargeState,setDQCurrent,setDQVoltage);
        tempStringSD.concat(tempStringSD2);
        myFile.printf(tempStringSD);
        SUPER_t_prev = Time.now();
      }
      myFile.close();
    }
  }
}

void ReadFrameLine(void){
  String fieldString;
  char fileLineChars[450];
  String fileLine;
  int i = 5;
  int j = 5;
  int n = 0; //number of chars read
  int tempSeconds = 0;
  String tempUserString;
  //static int frameNumber = 0;
  static int prevMillis = millis();
  static int nextFrameFound = 0;
  //static int attemptedOpen = 0;
  if (readFrameFileExists == 1){
    if (nextFrameFound == 0){                   // && (TimerExpired(&scastSDtimers[CT_SD_NEXT_LINE_DELAY])))
      //readFrameFileExists = openReadFrameFile(strReadFileName);
      n = readFramesFile.fgets(fileLineChars,450); //if no new #BMS frame found, read next line
      //Serial.print(fileLineChars);
      //Serial.println(n);
      //closeReadFrameFile();
      if (n > 0){
        fieldString = fileLineChars;

        if (fieldString.substring(0,4) == "#BMS"){
          //nextFrameFound = 1;
          while(fieldString.substring(i,i+1) != ","){  //---------seconds
            i++;
          }
          fileLine = fieldString.substring(j,i);
          tempSeconds = fileLine.toInt();
          //Serial.printf("frameDelaySecs: %d\n",tempSeconds);
          if (tempSeconds > 0){
            if (frameNumber == 0){
              CurrLoadFrame.Seconds = tempSeconds;
              NextLoadFrame.Seconds = tempSeconds;
            }
            else{
              CurrLoadFrame.Seconds = NextLoadFrame.Seconds;
              NextLoadFrame.Seconds = tempSeconds;
              nextFrameFound = 1;
            }
          }
          else{
            return;
          }
          if (frameNumber > 0){
            //PrevLoadFrame.Seconds = LoadFrame.Seconds-1;

            //Serial.printf("PrevframeDelaySecs: %d\n",CurrLoadFrame.Seconds);
            if (NextLoadFrame.Seconds > CurrLoadFrame.Seconds){
              frameDelaySec = NextLoadFrame.Seconds - CurrLoadFrame.Seconds;
            }
            else{
              frameDelaySec = 1;
            }

            if (frameDelaySec >10){
              frameDelaySec = 10;
            }
            frameDelaymSec = (frameDelaySec * 1000)-200;
            //Serial.println(frameDelaymSec);
            ResetSDTimer(CT_SD_NEXT_LINE_DELAY);

            //Serial.printf("FrameDelay: %d\n",frameDelaySec);
            //Serial.printf("millis: %d\n---------------------\n",(millis()-prevMillis));
            prevMillis = millis();
            //PrevLoadFrame = LoadFrame;
          }
          i++;
          while(fieldString.substring(i,i+1) != ","){  //-----------Battery Type
            i++;
            //Serial.println("type");
          }

          i++;
          while(fieldString.substring(i,i+1) != ","){  //-----------Fault Code
            i++;
            //Serial.println("fault");
          }
          j = i+1;

          i++;
          while(fieldString.substring(i,i+1) != ","){  //-----------Voltage
            i++;
            //Serial.println("volts");
          }
          fileLine = fieldString.substring(j,i);
          if (frameNumber == 0){
            CurrLoadFrame.Volts = fileLine.toFloat();
            NextLoadFrame.Volts = fileLine.toFloat();
          }
          else{
            CurrLoadFrame.Volts = NextLoadFrame.Volts;
            NextLoadFrame.Volts = fileLine.toFloat();
          }
          //LoadFrame.Volts = fileLine.toFloat();
          //Serial.println(LoadFrame.Volts);
          j = i+1;

          i++;
          while(fieldString.substring(i,i+1) != ","){  //-----------Current
            i++;
            //Serial.println("current");
          }
          fileLine = fieldString.substring(j,i);
          if (frameNumber == 0){
            CurrLoadFrame.Current = fileLine.toFloat();
            CurrLoadFrame.Power = CurrLoadFrame.Current * CurrLoadFrame.Volts;
            NextLoadFrame.Current = fileLine.toFloat();
            NextLoadFrame.Power = NextLoadFrame.Current * NextLoadFrame.Volts;
          }
          else{
            CurrLoadFrame.Current = NextLoadFrame.Current;
            CurrLoadFrame.Power = NextLoadFrame.Power;
            NextLoadFrame.Current = fileLine.toFloat();
            NextLoadFrame.Power = NextLoadFrame.Current * NextLoadFrame.Volts;
          }

          //LoadFrame.Current = fileLine.toFloat();
          //LoadFrame.Power = LoadFrame.Current * LoadFrame.Volts;
          //Serial.println(LoadFrame.Current);
          //SET EL LOAD TO DATA FRAME POWER LEVEL
          if(NextLoadFrame.Power>0){
            sendFixPwrLevel(0);
            IC1200Enable = 1;
            setIC1200Current = (NextLoadFrame.Power/battVoltage)+0.5;
            setIC1200Voltage = 32;
            if (battType == INVNTS_80AH){
              DQchargerEnable = 1;
              setDQVoltage = setIC1200Voltage;
              setDQCurrent = setIC1200Current;
              transmitRPDO1();
            }
            else{
              transmitIC1200VoltsAmps();
            }
          }
          else{
            IC1200Enable = 0;
            setIC1200Current = 0;
            setIC1200Voltage = 0;
            if (battType == INVNTS_80AH){
              DQchargerEnable = 0;
              setDQVoltage = setIC1200Voltage;
              setDQCurrent = setIC1200Current;
              transmitRPDO1();
            }
            else{
              transmitIC1200VoltsAmps();
            }
  
            sendFixPwrLevel((int)abs(NextLoadFrame.Power));
          }
          //Serial.println(frameDelaymSec);
          dispBatterySDPwr();

          frameNumber++;
        }
      }
      else{
        RestartFrameFileLines();
        tempUserString = String::format("#CYCLES,%d,%d",testSubCycleCount,testCycleCount);
        LogUserString(tempUserString);
        testSubCycleCount++;
        n = readFramesFile.fgets(fileLineChars,450);
        /*ResetSDTimer(CT_SD_CLOSE_DELAY);
        attemptedOpen = 0;
        frameNumber = 0;
        //ResetSDTimer(CT_SD_OPEN_DELAY);
        readFramesFile.seekSet(0);*/
        //Serial.println(fileLineChars);
        //n = readFramesFile.fgets(fileLineChars,450);
        //closeReadFrameFile();
      }
    }
    else{
      //TIMER EXPIRED ->
      if (TimerExpired(&scastSDtimers[CT_SD_NEXT_LINE_DELAY])){
        //SET NEXTFRAMEFOUND TO 0
        nextFrameFound = 0;
        //RESET TIMER
        //ResetSDTimer(CT_SD_NEXT_LINE_DELAY);
      }
    }
  }
  else {
    if ((attemptedOpen ==0)&&(TimerExpired(&scastSDtimers[CT_SD_CLOSE_DELAY]))){
      ResetSDTimer(CT_SD_OPEN_DELAY);
      //readFrameFileExists = openReadFrameFile(strReadFileName);
      attemptedOpen = 1;
    }
  }
}

void RestartFrameFileLines(void){
  ResetSDTimer(CT_SD_CLOSE_DELAY);
  attemptedOpen = 0;
  frameNumber = 0;
  readFramesFile.seekSet(0);
}

void ResetSDTimer(SD_TIMERS eSDTimer){
  if (eSDTimer == CT_SD_NEXT_LINE_DELAY){
    SetTimerWithMilliseconds(&scastSDtimers[eSDTimer], frameDelaymSec);
  }
  else if (eSDTimer < NUM_SD_TIMERS)
  {
    SetTimerWithMilliseconds(&scastSDtimers[eSDTimer], scawSDtimers[eSDTimer]);
  }
}
