//#include "defines.h"
#include "application.h"

//----------Externed Globals-----------------------------
extern float moduleSOCscale;
extern float battVoltage;
extern float battCurrent;
extern double maxDischargeCurrent;
extern float maxRegenCurrent;
extern int chargeStatus;
extern int BMSstatus;
extern int critDischAlarm;
extern int warningFlags;
extern float moduleMinTemperature;
extern float moduleMaxTemperature;
extern int battCell1mv;
extern int battCell2mv;
extern int battCell3mv;
extern int battCell4mv;
extern int battCell5mv;
extern int battCell6mv;
extern int battCell7mv;
extern int battCell8mv;
extern float moduleMaxMvolts;
extern float moduleMinMvolts;
extern int cellsDeltaV;
extern int battSN;
extern int underTempChargeStatus;
extern int underTempDischargeStatus;
extern int overTemperatureStatus;
extern int PCBAoverTempStatus;
extern int battTerminalOverTempStatus;
extern int overVoltageStatus;
extern int underVoltageStatus;
extern int overCurrentCharge;
extern int overCurrentDischarge;
extern int cellDeltaTempStatus;
extern int ModuleLostState;
extern int contactorMain;
extern int contactorCharge;


extern float ampHours;
extern float cumlAmpHrs;
extern float wattHours;
extern float cumlWattHrs;


extern int flagSDPause;
extern int CsmTemps[];

extern int heaterStatus;

extern int DQchargerEnable;
extern int IC1200Enable;
extern int DQoperational;
extern int DQwallPluggedIn;
extern int IC1200wallPluggedIn;
extern float DQvoltage;
extern float DQcurrent;
extern float IC1200voltage;
extern float IC1200current;
extern float setIC1200Voltage;
extern float setIC1200Current;

extern float setDQVoltage;
extern float setDQCurrent;

extern int DQfixedCurrOverride;
extern int DQfixedVoltOverride;

extern int SDcardInitOK; //used for SD card status indication
extern int dischargeListFromSD;

extern int keySignalStatus;
extern int rechargeState;

extern int cumminsSystemState;
extern int cumminsDischrgEnabled;
extern int cumminsInterlockDetected;

extern int battType;

extern int readFrameFileExists;
extern int myFileOk;
//extern int frameFileOk;

extern int SparkNetOk;

extern unsigned int testSubCycleCount;
extern unsigned long testCycleCount;

extern int testType;
extern int dischargerType;
extern float sdLogRate;
extern int what2logTxt;

extern int onceThrough;   //init tasks in periodic scheduler

extern double BMSchargeCurrSetpoint;
extern double BMSchargeVoltSetpoint;

extern unsigned int BMSstatusWord;

extern int InvntsSystemState;
extern int InvntsDischrgEnabled;
extern int InvntsInterlockDetected;
extern int InvntsSOH;
extern int InvntsCANRec;

extern int overTempChargeStatus;
extern int otherChargeFaultStatus;
extern int overTempDischargeStatus;
extern int otherDischargeFaultStatus;
extern int shortCircuitStatus;
extern int internalCommStatus;

extern int InvntsHeaterStat;

extern int maxChargePercent;
extern int minDischargePercent;

extern int tempOk2ChargeStatus;

extern int Invnts_ATSAMfirmwareLetterRev;
extern int Invnts_ATSAMfirmwareMajorRev;
extern int Invnts_ATSAMfirmwareMinorRev;
extern int Invnts_BQ80firmwareLetterRev;
extern int Invnts_BQ80firmwareMajorRev;
extern int Invnts_BQ80firmwareMinorRev;
