#include "CycleTest.h"
#include "TickTimer.h"
#include "application.h"
#include "Log2SdCard.h"
#include "defines.h"
#include "Serial_El_Load.h"
#include "Globals.h"
#include "Rev3Messages.h"
#include "CANtx.h"
#include "DeltaQ_CANopen.h"
#include "CumminsCAN.h"
#include "InvntsOldCAN.h"
#include "Invnts60AhCAN.h"
#include "Invnts80AhCAN.h"
#include "InvntsVirtualBattCAN.h"

static const unsigned int scawCycleTestTimers[NUM_CYCLE_TIMERS] =
{
  /* CT_KEY_RESTART_DELAY     */   2000,  //ms -TIME TO KEEP THE IGNITION SIGNAL off
  /* CT_KEY_WAKE              */   9000,  //ms -TIME TO WAIT AFTER KEY ON SIGNAL
  /* CT_EL_LOAD_CMD_DELAY     */   200,   //ms -TIME TO WAIT BETWEEN COMMANDS TO THE EL. LOAD
  /* CT_MAIN_CNTCTR_DELAY     */   2000,  //ms -TIME LAG OF MAIN CONTACTOR STATE REPORTING ON CANBUS
  /* CT_RECHARGE_DELAY        */   8000,  //ms -MAX TIME TO WAIT AFTER POWERING ON CHARGER -CUMMINS ONLY
  /* CT_CUMMINS_CHRG_DELAY    */   6000,  //ms -MIN TIME TO WAIT AFTER POWERING ON CHARGER -CUMMINS ONLY -MUST BE SHORTER THAN CT_RECHARGE_DELAY
  /* CT_LOG_INTERVAL          */   4000,  //ms -TIME TO WAIT BETWEEN COMMANDS TO THE EL. LOAD
  /* CT_INVNTS_DISCHRG_DELAY  */   1000   //ms -TIME TO WAIT BEFORE ASSUMING DISCHARGE LIMIT IS ACTUALLY ZERO
};

stateTestControl testState;
stateTestControl testStatePrev;
stateTestControl testStateKeyOnHold;
stateTestControl testStateCopy;
heaterStateControl heaterState;
heaterStateControl heaterStateCopy;

int keyRestarting = 0;
int holdCharge = 0;
int heaterCommandOverride = 0;
int prevDQCurr = 0;
float fixedOverRideCurr = 0;
float fixedOverRideVolt = 0;

static TICK_TIMER_S scastCycleTimers[NUM_CYCLE_TIMERS];

//exposed Particle variables
String testState2String = "stateINIT";
String battCurr2String;
String battMinTemp2String;
String battMaxTemp2String;
String battSOC2String;
String battVolt2String;
String ok2ChargeStatus;
String ok2DischargeStatus;
String heaterState2String = "heaterOFF";
String heaterStatus2String;
String cellsDeltaV2String;
String cellMinVoltsString;
String cellMaxVoltsString;
String cycleCountString;
String subCycleCountString;

float maxSurfTempReturned = -99;

void ResetCycleTimer(CYCLE_TIMERS eCycleTimer);
void keyRestart(void);
void manageKeyRestart(void);
void rechargeOn(void);
void rechargeOff(void);
int fStateTest(String command);
int fStateHeater(String heaterCommand);
int fSetChrgCurr(String setCurrString);
int fSetChrgVolt(String setVoltString);
int fElLoadFunc(String command);
int fSetMinDischargePercent(String minDischargePrcnt);
int fSetMaxChargePercent(String maxChargePrcnt);

int okToDischarge(void);
int okToCharge(void);
void turnHeaterOn(void);
void turnHeaterOff(void);
void manageHeaterEnabled(void);
float maxCSMSurfaceTemp(void);
void manageSDLogging(void);

int fElLoadString(String command){
  Serial1.println(command);
  //Serial.println(command);
  return 1;
}


void initCycleTest(void){
  unsigned int wIndex;
  for (wIndex = 0; wIndex < ((unsigned int)NUM_CYCLE_TIMERS); wIndex++)
  {
    RegisterTimer(&scastCycleTimers[wIndex]);
    ResetCycleTimer((CYCLE_TIMERS)wIndex);
  }
  initSerialElLoad();
  Particle.function("ForceState",fStateTest);
  Particle.function("ForceHeater",fStateHeater);
  Particle.function("SetChrgCurr",fSetChrgCurr);
  Particle.function("SetChrgVolt",fSetChrgVolt);
  Particle.function("ElLoadString",fElLoadString);
  Particle.function("SetMinDischargePercent",fSetMinDischargePercent);
  Particle.function("SetMaxChargePercent",fSetMaxChargePercent);
  
  Particle.variable("OkToCharge", ok2ChargeStatus);
  Particle.variable("okToDischarge", ok2DischargeStatus);
  Particle.variable("CycleCount", testCycleCount);
  Particle.variable("HeaterState", heaterState2String);
  Particle.variable("HeaterStatus", heaterStatus2String);
  Particle.variable("StateMachine", testState2String);
  Particle.variable("BattVoltage", battVolt2String);
  Particle.variable("BattCurrent", battCurr2String);
  Particle.variable("SOC", battSOC2String);
  Particle.variable("CellMinTemp", battMinTemp2String);
  Particle.variable("CellMaxTemp", battMaxTemp2String);
  //Particle.variable("CellDeltaV", cellsDeltaV2String);
  Particle.variable("CellMinVolts", cellMinVoltsString);
  Particle.variable("CellMaxVolts", cellMaxVoltsString);
  Particle.variable("chargeVsetpoint", BMSchargeVoltSetpoint);
  Particle.variable("chargeAsetpoint", BMSchargeCurrSetpoint);
  Particle.variable("maxDischargeCurr", maxDischargeCurrent);

  
  pinMode(CHARGE_EN, OUTPUT);
  pinMode(IGN, OUTPUT);
  if (battType == VALENCE_REV3){
    pinMode(HEATER, OUTPUT);
  }
  else if (battType == CUMMINS_REV1){
    pinMode(BRAMMO_INTRLK, OUTPUT);
  }
  else if (battType == INVNTS_OLD){
    pinMode(INVNTS_DISCHRG, OUTPUT);
  }
  else if (battType == INVNTS_60AH){
    pinMode(INVNTS_DISCHRG, OUTPUT);
  }
  else if (battType == INVNTS_80AH){
    pinMode(INVNTS_DISCHRG, OUTPUT);
  }
  else if (battType == INVNTS_VIRT_BATT){
    pinMode(INVNTS_DISCHRG, OUTPUT);
  }

  testState = stateINIT;

  testSubCycleCount = 0;
  testCycleCount = 0;
}

void CycleTest(void){
  //manageKeyRestart();
  testState2String = stateStrings[testState]; //set the exposed particle variable to the test state string
  heaterState2String = heaterStateStrings[heaterState]; //set the exposed particle variable to the heater state string
  //float fCellsDeltaV = (float)(cellsDeltaV/1000);
  //cellsDeltaV2String = String::format("%d",cellsDeltaV);
  cellMinVoltsString = String::format("%.3f", moduleMinMvolts);
  cellMaxVoltsString = String::format("%.3f", moduleMaxMvolts);
  battMaxTemp2String = String::format("%.1f", moduleMaxTemperature);
  battMinTemp2String = String::format("%.1f", moduleMinTemperature);
  //cellsDeltaV = moduleMaxMvolts - moduleMinMvolts;
  battSOC2String = String::format("%.2f", moduleSOCscale);
  battVolt2String = String::format("%.3f", battVoltage);
  battCurr2String = String::format("%.1f", battCurrent);
  cycleCountString = String::format("%d", testCycleCount);
  subCycleCountString = String::format("%d", testSubCycleCount);

  float DQcurrentSetpoint = 0;

  int tempOk2Discharge = 0;

  char Invnts_ATSAMfirmwareLetterRev_char = 0; 
  char Invnts_ATSAMfirmwareMajorRev_char = 0;
  char Invnts_ATSAMfirmwareMinorRev_char = 0;
  String Invnts_ATSAMfirmwareRev = "#ATSAM Version,";

  char Invnts_BQ80firmwareLetterRev_char = 0;
  char Invnts_BQ80firmwareMajorRev_char = 0;
  char Invnts_BQ80firmwareMinorRev_char = 0;
  String Invnts_BQ80firmwareRev = "#BQ80 Version,";

  String BMSstatusString = "";

  maxSurfTempReturned = maxCSMSurfaceTemp();

  manageSDLogging();

  if (heaterStatus == 0){
    heaterStatus2String = "OFF";
  }
  else if (heaterStatus == 1){
    heaterStatus2String = "ON";
  }
  else{
    heaterStatus2String = "ERROR";
  }

  tempOk2ChargeStatus = okToCharge();
  ok2ChargeStatus = String::format("%d",tempOk2ChargeStatus);

  int tempOk2DischargeStatus = okToDischarge();
  ok2DischargeStatus = String::format("%d",tempOk2DischargeStatus);

  if (battType == VALENCE_REV3){
    //-------------- HEATER SAFETY SHUTDOWN CHECK ----------------
    if (maxSurfTempReturned >= HEATER_OFF_SURFACE_TEMP){      //--
      turnHeaterOff();                                        //--
    }                                                         //--
    if (maxSurfTempReturned > HEATER_OFF_CRIT_TEMP){          //--
      digitalWrite(HEATER, LOW);                              //--
      heaterStatus = 0;                                       //--
      setDQVoltage = 0;                                       //--
      heaterState = heaterError;                              //--
      testState = stateERRORHALT;                             //--
    }                                                         //--
    //------------------------------------------------------------
  }
  //Serial.println(testState2String);
  switch (testState)
  {
    case stateINIT:             //0
      flagSDPause = 1;
      DQchargerEnable = 1;

      IC1200Enable = 0;
      setIC1200Current = 0;
      setIC1200Voltage = 0;

      DQfixedCurrOverride = 0;
      DQfixedVoltOverride = 0;
      
      testState = statePOWER_ON;
      

      if(battType == VALENCE_REV3){
        heaterState = heaterOFF;
        turnHeaterOff();
        rechargeOff();
        keyRestart();
      }
      else if (battType == CUMMINS_REV1){
        heaterState = heaterDisabled;
        digitalWrite(BRAMMO_INTRLK, LOW);
        rechargeOff();
      }
      else if (battType == INVNTS_OLD){
        heaterState = heaterDisabled;
        digitalWrite(INVNTS_DISCHRG, HIGH);
        digitalWrite(CHARGE_EN, HIGH);
        rechargeOff();
      }
      else if (battType == INVNTS_60AH){
        heaterState = heaterDisabled;
        digitalWrite(INVNTS_DISCHRG, HIGH);
        digitalWrite(CHARGE_EN, HIGH);
        rechargeOff();
      }
      else if (battType == INVNTS_80AH){
        heaterState = heaterDisabled;
        digitalWrite(INVNTS_DISCHRG, HIGH);
        digitalWrite(CHARGE_EN, HIGH);
        rechargeOff();
      }
      else if (battType == INVNTS_VIRT_BATT){
        heaterState = heaterDisabled;
        digitalWrite(INVNTS_DISCHRG, HIGH);
        digitalWrite(CHARGE_EN, LOW);
        rechargeOff();
      }

      if (readFrameFileExists == 1){
        LogUserString("#CYCLES,#PH_CYCLES,#RECHARGE_CYCLES");
      }
      break;

    case statePOWER_ON:          //1
      //transmitDQGoOp();
      initSerialElLoad();
      putElLoadIntoRemote();
      testState = statePWRRESET_LOAD_OFF;
      break;

    case statePWRRESET_LOAD_OFF:  //2
      //turn off electronic load
      turnElLoadOFF();
      setElLoadToFixed();
      if (battType == CUMMINS_REV1){
        heaterState = heaterDisabled;
        digitalWrite(BRAMMO_INTRLK, LOW);
        rechargeOn();
      }
      else if (battType == INVNTS_OLD){
        //heaterState = heaterDisabled;
      }
      else if (battType == INVNTS_60AH){
        //heaterState = heaterDisabled;
      }
      else if (battType == INVNTS_80AH){
        //heaterState = heaterDisabled;
      }
      else if (battType == INVNTS_VIRT_BATT){
        //heaterState = heaterDisabled;
      }

      testState = statePOWERRESETCHRGON;
      
      break;

    case statePOWERRESETCHRGON:   //3
      if (battType != INVNTS_OLD){
        IC1200Enable = 0;
        setIC1200Current = 0;
        setIC1200Voltage = 0;
      }
      else if (battType != INVNTS_60AH){
        IC1200Enable = 0;
        setIC1200Current = 0;
        setIC1200Voltage = 0;
      }
      else if (battType != INVNTS_80AH){
        IC1200Enable = 0;
        setIC1200Current = 0;
        setIC1200Voltage = 0;
      }
      else if (battType != INVNTS_VIRT_BATT){
        IC1200Enable = 0;
        setIC1200Current = 0;
        setIC1200Voltage = 0;
      }
      else{
        if (InvntsCANRec == 0){
          break;
        }
      }

      //Serial.println(okToCharge());
      if (battType == VALENCE_REV3){
        if (BMSrev3CANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = statePOWERRESET_CHARGE;  //start charging
            }
            else if(okToCharge() == 11){ //too cold to charge yet
              rechargeOff();
              testState = statePOWERRESET_CHARGE;
            }
            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
        else {
          rechargeOff();
          testState = stateERRORHALT;
        }
      }
      else if (battType == CUMMINS_REV1){
        if (TimerExpired(&scastCycleTimers[CT_CUMMINS_CHRG_DELAY])){
          if (TimerRunning(&scastCycleTimers[CT_RECHARGE_DELAY])){
            if (DQwallPluggedIn == 1){
              if (moduleSOCscale < maxChargePercent){
                testState = statePOWERRESET_CHARGE;
              }
              else{
                testState = statePOWERRESETCHRGOFF;
              }
            }
          }
          else{
            testState = statePOWERRESETCHRGOFF;
          }
        }
      }
      else if (battType == INVNTS_OLD){
        if (InvntsOldCANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = statePOWERRESET_CHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = statePOWERRESET_CHARGE;
            }

            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
        else {
          rechargeOff();
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_60AH){
        if (Invnts60AhCANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = statePOWERRESET_CHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = statePOWERRESET_CHARGE;
            }

            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
        else {
          rechargeOff();
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_80AH){
        if (Invnts80AhCANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = statePOWERRESET_CHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = statePOWERRESET_CHARGE;
            }

            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
        else {
          rechargeOff();
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_VIRT_BATT){
        if (InvntsVirtualBattCANok()){

          if ((Invnts_ATSAMfirmwareLetterRev != 0)&&(Invnts_BQ80firmwareLetterRev != 0)){
            Invnts_ATSAMfirmwareLetterRev_char = Invnts_ATSAMfirmwareLetterRev; 
            Invnts_ATSAMfirmwareMajorRev_char = Invnts_ATSAMfirmwareMajorRev;
            Invnts_ATSAMfirmwareMinorRev_char = Invnts_ATSAMfirmwareMinorRev;
            Invnts_ATSAMfirmwareRev.concat(Invnts_ATSAMfirmwareLetterRev_char);
            Invnts_ATSAMfirmwareRev.concat(Invnts_ATSAMfirmwareMajorRev_char);
            Invnts_ATSAMfirmwareRev.concat(".");
            Invnts_ATSAMfirmwareRev.concat(Invnts_ATSAMfirmwareMinorRev_char);
            
            Invnts_BQ80firmwareLetterRev_char = Invnts_BQ80firmwareLetterRev;
            Invnts_BQ80firmwareMajorRev_char = Invnts_BQ80firmwareMajorRev;
            Invnts_BQ80firmwareMinorRev_char = Invnts_BQ80firmwareMinorRev;
            Invnts_BQ80firmwareRev.concat(Invnts_BQ80firmwareLetterRev_char);
            Invnts_BQ80firmwareRev.concat(Invnts_BQ80firmwareMajorRev_char);
            Invnts_BQ80firmwareRev.concat(".");
            Invnts_BQ80firmwareRev.concat(Invnts_BQ80firmwareMinorRev_char);

            LogUserString(Invnts_ATSAMfirmwareRev);
            LogUserString(Invnts_BQ80firmwareRev);

            //Particle.publish("ATSAM", Invnts_ATSAMfirmwareRev, PRIVATE);
            //Particle.publish("BQ80", Invnts_BQ80firmwareRev, PRIVATE);

          }
          else{
            break;
          }

          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = statePOWERRESET_CHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = statePOWERRESET_CHARGE;
            }

            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
        else {
          //rechargeOff();
          //testState = stateERRORHALT;
        }
      }
      else{
        testState = stateERRORHALT;
      }
      break;

    case statePOWERRESET_CHARGE: //4
      flagSDPause = 0;
      if (battType==VALENCE_REV3){
        if (chargeStatus==COMPLETE){ //(moduleSOCscale > 95.0)
          testState = statePOWERRESETCHRGOFF;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 11){
          rechargeOff();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType==CUMMINS_REV1){
        if((moduleSOCscale==100)||(DQwallPluggedIn == 0)){
          testState = statePOWERRESETCHRGOFF;
        }
      }
      else if (battType == INVNTS_OLD){
        if (moduleSOCscale >= maxChargePercent){
          testState = statePOWERRESETCHRGOFF;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_60AH){
        if (moduleSOCscale >= maxChargePercent){
          testState = statePOWERRESETCHRGOFF;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_80AH){
        if ((moduleSOCscale >= maxChargePercent)||(okToCharge() == 6)){
          testState = statePOWERRESETCHRGOFF;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_VIRT_BATT){
        if ((moduleSOCscale >= 100)||(okToCharge() == 6)){
          testState = statePOWERRESETCHRGOFF;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      break;

    case statePOWERRESETCHRGOFF:  //5
      flagSDPause = 0;
      rechargeOff();
      testState = stateSTARTDISCHARGE;
      if (battType == INVNTS_OLD){
        if (okToDischarge() == 15){
          testState = statePOWERRESETCHRGOFF;
        }
      }
      else if (battType == INVNTS_60AH){
        if (okToDischarge() == 15){
          testState = statePOWERRESETCHRGOFF;
        }
      }
      else if (battType == INVNTS_80AH){
        if (okToDischarge() == 15){
          testState = statePOWERRESETCHRGOFF;
        }
      }
      else if (battType == INVNTS_VIRT_BATT){
        if (okToDischarge() == 15){
          testState = statePOWERRESETCHRGOFF;
        }
      }
      break;

    case stateSTARTDISCHARGE:     //6
      cumlAmpHrs = 0;
      cumlWattHrs = 0;
      if (okToDischarge() == 15){
        break;
      }
      if (dischargerType == SD_REPLAY){
        //configure discharger for replicating list read from SD card
        RestartFrameFileLines();
        testSubCycleCount = 1;
        setElLoad2PwrMode();
        ReadFrameLine();  //begin reading data frames from the SD card
        testState = stateDISCHARGE;
      }
      else {
        if (startListRunning()){  //when function returns a 1, all messages have been sent
          testState = stateDISCHARGE;
        }
      }
      if (battType == CUMMINS_REV1){
        digitalWrite(BRAMMO_INTRLK, HIGH);
      }
      break;

    case stateDISCHARGE:          //7
      flagSDPause = 0;
      //Serial.println("discharge");
      if (dischargerType == SD_REPLAY){
        ReadFrameLine();  //try to read the next data frame from the SD card
      }
      if(battType == VALENCE_REV3){
        if ((okToDischarge() == 12)||(okToDischarge() == 2)){
          break;
        }
        else if (okToDischarge() != 1){
          testState = stateDISCHARGE_INPUTOFF;
        }
      }
      else if (battType == CUMMINS_REV1){
        if ((!CumminsCANok())||(moduleSOCscale<1.0)){
          digitalWrite(BRAMMO_INTRLK, LOW);
          testState = stateDISCHARGE_INPUTOFF;
        }
      }
      else if(battType == INVNTS_OLD){
        //Serial.println(okToDischarge());
        if (okToDischarge() != 1){
          testState = stateDISCHARGE_INPUTOFF;
        }
      }  
      else if(battType == INVNTS_60AH){
        //Serial.println(okToDischarge());
        if (okToDischarge() != 1){
          testState = stateDISCHARGE_INPUTOFF;
        }
      }
      else if(battType == INVNTS_80AH){
        //Serial.println(okToDischarge());
        
        if (okToDischarge() != 1){

          testState = stateDISCHARGE_INPUTOFF;
        }
      }
      else if(battType == INVNTS_VIRT_BATT){
        //Serial.println(okToDischarge());
        //tempOk2DischargeStatus = okToDischarge();

        BMSstatusString = String::format("%d", BMSstatusWord);
        
        if (okToDischarge() != 1){
          //Particle.publish("OK_2_Discharge_Val", ok2DischargeStatus, PRIVATE);
          //Particle.publish("BMS_Status", BMSstatusString, PRIVATE);
          if(okToDischarge() == 15){
            if (TimerExpired(&scastCycleTimers[CT_INVNTS_DISCHRG_DELAY])){
              testState = stateDISCHARGE_INPUTOFF;
            }
          }
          else{
            testState = stateDISCHARGE_INPUTOFF;
          }
        }
        else {
          ResetCycleTimer(CT_INVNTS_DISCHRG_DELAY);
        }
      }
      break;

    case stateDISCHARGE_INPUTOFF: //8
      flagSDPause = 1;
      turnElLoadOFF();
      setElLoadToFixed();
      testState = stateCHARGEENABLE;
      if(battType == VALENCE_REV3){
        keyRestart();
      }
      else if (battType == CUMMINS_REV1){
        digitalWrite(BRAMMO_INTRLK, LOW);
        rechargeOn();
      }
      else if (battType == INVNTS_OLD){
        //digitalWrite(INVNTS_DISCHRG, LOW);
        rechargeOn();
      }
      else if (battType == INVNTS_60AH){
        //digitalWrite(INVNTS_DISCHRG, LOW);
        rechargeOn();
      }
      else if (battType == INVNTS_80AH){
        //digitalWrite(INVNTS_DISCHRG, LOW);
        rechargeOn();
      }
      else if (battType == INVNTS_VIRT_BATT){
        //digitalWrite(INVNTS_DISCHRG, LOW);
        rechargeOn();
      }
      break;

    case stateCHARGEENABLE:       //9
      IC1200Enable = 0;
      setIC1200Current = 0;
      setIC1200Voltage = 0;

      if(battType == VALENCE_REV3){
        if (BMSrev3CANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = stateCHARGE;  //start charging
            }
            else if(okToCharge() == 11){ //too cold to charge yet
              rechargeOff();
              testState = stateCHARGE;
            }
            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
        else {
          rechargeOff();
          testState = stateERRORHALT;
        }
      }
      else if (battType == CUMMINS_REV1){
        if (TimerRunning(&scastCycleTimers[CT_RECHARGE_DELAY])){
          if (DQwallPluggedIn == 1){
            if (TimerExpired(&scastCycleTimers[CT_CUMMINS_CHRG_DELAY])){
              testState = stateCHARGE;
            }
          }
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_OLD){
        if (InvntsOldCANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = stateCHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = stateCHARGE;
            }
            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
      }
      else if (battType == INVNTS_60AH){
        if (Invnts60AhCANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = stateCHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = stateCHARGE;
            }
            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
      }
      else if (battType == INVNTS_80AH){
        if (Invnts80AhCANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = stateCHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = stateCHARGE;
            }
            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
      }
      else if (battType == INVNTS_VIRT_BATT){
        if (InvntsVirtualBattCANok()){
          if (moduleSOCscale < maxChargePercent){
            if (okToCharge() == 1){     //ok to charge
              rechargeOn();
              testState = stateCHARGE;  //start charging
            }
            else if(okToCharge() == 2){ //too cold to charge yet
              rechargeOff();
              testState = stateCHARGE;
            }
            else{
              rechargeOff();
              testState = stateERRORHALT; //something is wrong that prevents charging
            }
          }
          else {
            rechargeOff();
            testState = stateSTARTDISCHARGE;
          }
        }
      }
      break;

    case  stateCHARGE:                //10
      flagSDPause = 0;
      if (battType==VALENCE_REV3){
        if (chargeStatus==COMPLETE){    //(moduleSOCscale > 95.0)
          testState = stateENDOFCHRG;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 11){
          rechargeOff();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType==CUMMINS_REV1){
        if((moduleSOCscale==100.0)||(DQwallPluggedIn == 0)){
          testState = stateENDOFCHRG;
        }
      }
      else if (battType == INVNTS_OLD){
        if (moduleSOCscale >= maxChargePercent){  //(chargeStatus==COMPLETE) //(moduleSOCscale > 95.0)
          testState = stateENDOFCHRG;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_60AH){
        if (moduleSOCscale >= maxChargePercent){  //(chargeStatus==COMPLETE) //(moduleSOCscale > 95.0)
          testState = stateENDOFCHRG;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_80AH){
        if ((moduleSOCscale >= maxChargePercent)||(okToCharge() == 6)){  //(chargeStatus==COMPLETE) //(moduleSOCscale > 95.0)
          testState = stateENDOFCHRG;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      else if (battType == INVNTS_VIRT_BATT){
        if ((moduleSOCscale >= maxChargePercent)||(okToCharge() == 6)){  //(chargeStatus==COMPLETE) //(moduleSOCscale > 95.0)
          testState = stateENDOFCHRG;
        }
        else if (okToCharge() == 1){
          rechargeOn();
        }
        else if (okToCharge() == 2){
          rechargeOn();
        }
        else{
          testState = stateERRORHALT;
        }
      }
      break;

    case  stateENDOFCHRG:             //11
      rechargeOff();
      testState = stateREPORTOUT;
      if (battType == INVNTS_OLD){
        if (okToDischarge() == 15){
          testState = stateENDOFCHRG;
        }
      }
      if (battType == INVNTS_60AH){
        if (okToDischarge() == 15){
          testState = stateENDOFCHRG;
        }
      }
      if (battType == INVNTS_80AH){
        if (okToDischarge() == 15){
          testState = stateENDOFCHRG;
        }
      }
      if (battType == INVNTS_VIRT_BATT){
        if (okToDischarge() == 15){
          testState = stateENDOFCHRG;
        }
      }
      break;

    case  stateREPORTOUT:             //12
      cumlAmpHrs = 0;
      cumlWattHrs = 0;
      testCycleCount++;
      testState = stateSTARTDISCHARGE;
      break;

    case  stateCOMMANDHALT:           //13
      flagSDPause = 1;
      turnElLoadOFF();
      setElLoadToFixed();
      if (battType == INVNTS_VIRT_BATT){
        digitalWrite(INVNTS_DISCHRG, LOW);
        digitalWrite(CHARGE_EN, LOW);
      }
      rechargeState = 0;
      //testState = stateHALT;
      break;

    case  stateERRORHALT:             //14
      flagSDPause = 1;
      turnElLoadOFF();
      setElLoadToFixed();
      if (battType == INVNTS_VIRT_BATT){
        digitalWrite(INVNTS_DISCHRG, LOW);
        digitalWrite(CHARGE_EN, LOW);
      }
      rechargeState = 0;
      //testState = stateHALT;
      break;

    case  stateHALT:                  //15
      flagSDPause = 1;
      turnElLoadOFF();
      setElLoadToFixed();
      if (battType == INVNTS_VIRT_BATT){
        digitalWrite(INVNTS_DISCHRG, LOW);
        digitalWrite(CHARGE_EN, LOW);
      }
      rechargeState = 0;
      break;

    case  statePAUSE:                 //16
      break;

    case stateKEYRESTART:             //17
      //Serial.printf("key restart: %d\n", wMsTimerWillExpireIn(&scastCycleTimers[CT_KEY_RESTART_DELAY]));
      //Serial.printf("key wake: %d\n", wMsTimerWillExpireIn(&scastCycleTimers[CT_KEY_WAKE]));
      //Serial.printf("key restart expired: %d\n", testState);

      if (keyRestarting == 0)
      {
        keyRestart();
      }
      else if (keyRestarting == 1){
        manageKeyRestart();
      }
      else{
        testState = testStateKeyOnHold;
        keyRestarting = 0;
      }
      break;
  }
  testStateCopy = testState;

//change delta-q settings to additive current based on known loads and expected voltage
  //----------------------------   MANAGE DELTA-Q SETTINGS    -------------------------------------------------------//
  DQcurrentSetpoint = 0.0;
  if ((battType == INVNTS_VIRT_BATT)&&(InvntsHeaterStat==1)){   //need to put the correct DQ handshaking here
    //DQcurrentSetpoint += 8;
  }
  else {
    if (heaterStatus == 1){
      DQcurrentSetpoint += 2; //7
    }
  }

  if (0){  //set dq current setpoint based on cell voltage and temps
    if ((testState == statePOWERRESET_CHARGE) || (testState == stateCHARGE)){
      if (moduleMaxMvolts >= CELL_MAX_VOLTS_H){
        DQcurrentSetpoint += 4;
        setDQVoltage = 30.4;
      }
      else if (moduleMaxMvolts < CELL_MAX_VOLTS_L){
        DQcurrentSetpoint += 34;
        setDQVoltage = 30.4;
      }
      else{
        DQcurrentSetpoint += 15;
        setDQVoltage = 30.4;
      }
      if ((moduleMaxTemperature > MODULE_HIGH_TEMP_CHRG_CUTBACK_ON)&&(DQcurrentSetpoint > 15)){
        DQcurrentSetpoint = 15;
      }
    }
  }
  if (testState != stateDISCHARGE){  //set dq current setpoint based on BMS current request, don't override the SD card playback value for regen
    DQcurrentSetpoint = DQcurrentSetpoint + BMSchargeCurrSetpoint;
  }
  if ((moduleMaxTemperature > MODULE_HIGH_TEMP_CHRG_CUTBACK_ON)&&(DQcurrentSetpoint > 15)){
    DQcurrentSetpoint = 15;
  }

  if (DQcurrentSetpoint >= 34){
    DQcurrentSetpoint = 34;
  }

  if (testState != stateDISCHARGE){  //don't override the SD card playback value for regen
    if (battType == INVNTS_OLD){
      if((moduleMaxMvolts>3550)&&(DQcurrentSetpoint > 7)){
        DQcurrentSetpoint = 7;
      }
      if((moduleMaxMvolts>3520)&&(DQcurrentSetpoint > 15)){
        DQcurrentSetpoint = 15;
      }
      if (moduleMaxMvolts<3490){
        DQcurrentSetpoint = BMSchargeCurrSetpoint;
      }
    }
    else if (battType == INVNTS_60AH){
      if((moduleMaxMvolts>3550)&&(DQcurrentSetpoint > 7)){
        DQcurrentSetpoint = 7;
      }
      if((moduleMaxMvolts>3520)&&(DQcurrentSetpoint > 15)){
        DQcurrentSetpoint = 15;
      }
      if (moduleMaxMvolts<3490){
        DQcurrentSetpoint = BMSchargeCurrSetpoint;
      }
    }
    else if (battType == INVNTS_80AH){
      if((moduleMaxMvolts>3550)&&(DQcurrentSetpoint > 7)){
        DQcurrentSetpoint = 7;
      }
      if((moduleMaxMvolts>3520)&&(DQcurrentSetpoint > 15)){
        DQcurrentSetpoint = 15;
      }
      //if (moduleMaxMvolts<3490){                    //commented out to allow heater offset to function correctly
        //DQcurrentSetpoint = BMSchargeCurrSetpoint;
      //}
    }
    else if (battType == INVNTS_VIRT_BATT){
      if((moduleMaxMvolts>3520)&&(DQcurrentSetpoint > 7)){
        DQcurrentSetpoint = 4;
      }
      if((moduleMaxMvolts>3500)&&(DQcurrentSetpoint > 15)){
        DQcurrentSetpoint = 12;
      }
      //if (moduleMaxMvolts<3490){                    //commented out to allow heater offset to function correctly
        //DQcurrentSetpoint = BMSchargeCurrSetpoint;
      //}
    }
    setDQCurrent = DQcurrentSetpoint;
    setDQVoltage = BMSchargeVoltSetpoint;
  }
  

  //--------------------handle Particle function override command-----------------
  if (DQfixedCurrOverride == 1){
    setDQCurrent = fixedOverRideCurr;
  }
  if (DQfixedVoltOverride == 1){
    setDQVoltage = fixedOverRideVolt;
  }
  

  //----------------------------   MANAGE HEATER STATE MACHINE   ----------------------------------------------------//
  switch(heaterState)
  {
    case heaterOFF: //------------------------------------
      turnHeaterOff();
      if ((heaterCommandOverride == 0)&&(moduleMinTemperature<CHRGR_PWR_HEATER_OFF_CELL_TEMP)){  //heater state is not forced and temp is cold enough to worry about
        if (DQwallPluggedIn){                                               //DeltaQ has wall power
          if (moduleMinTemperature <= CHRGR_PWR_HEATER_ON_CELL_TEMP){       //min battery cell temp is less than turn-on thresh
            heaterState = heaterON_Charging;
          }
        }
        else{                                                               //DeltaQ has no wall power
          if ((moduleMinTemperature <= CHRGR_OFF_HEATER_ON_CELL_TEMP) && (moduleSOCscale > BATT_SOC_HEATER_MIN_THRESH)){ //min battery cell temp is less than turn-on thresh and SOC is higher than thresh
            heaterState = heaterONnotCharging;
          }
        }
      }
      break;
    case heaterForceOn: //----------------------  force heater to try to run, within surface temperature bounds
      if (heaterCommandOverride == 1){
        manageHeaterEnabled();
      }
      else{
        heaterState = heaterOFF;
      }
      break;
    case heaterONnotCharging: //--------------------------- MANAGE THE HEATER ON/OFF AND KICKOUT WHEN MODULE CELLS ARE WARMED UP
      if (!DQwallPluggedIn){
        if ((moduleMinTemperature <= CHRGR_OFF_HEATER_OFF_CELL_TEMP) && (moduleSOCscale > BATT_SOC_HEATER_MIN_THRESH)){
          manageHeaterEnabled();
        }
        else{
          heaterState = heaterOFF;
        }
      }
      else{
        heaterState = heaterOFF;
      }
      break;
    case heaterON_Charging: //----------------------------- MANAGE THE HEATER ON/OFF AND KICKOUT WHEN MODULE CELLS ARE WARMED UP
      if (DQwallPluggedIn){
        if (moduleMinTemperature <= CHRGR_PWR_HEATER_OFF_CELL_TEMP){
          manageHeaterEnabled();
        }
        else{
          heaterState = heaterOFF;
        }
      }
      else{
        heaterState = heaterOFF;
      }
      break;
    case heaterError: //------------------------------------  TRAP IN HERE IF ERROR RUNNING HEATER
      turnHeaterOff();
      break;
    case heaterDisabled:  //---------------------------------   IF CUMMINS BATTERY, HEATER CONTROL ISN'T NEEDED
      break;
  }
}

void keyRestart(void)
{
  digitalWrite(IGN, LOW);
  keySignalStatus = 0;
  ResetCycleTimer(CT_KEY_RESTART_DELAY);
  keyRestarting = 1;
  testStateKeyOnHold = testState;
  testState = stateKEYRESTART;
}

void manageKeyRestart(void)
{
  if (TimerRunning(&scastCycleTimers[CT_KEY_RESTART_DELAY]))
  {
    ResetCycleTimer(CT_KEY_WAKE);
  }
  else if ((TimerExpired(&scastCycleTimers[CT_KEY_RESTART_DELAY])) && (TimerRunning(&scastCycleTimers[CT_KEY_WAKE])))
  {
    digitalWrite(IGN, HIGH);
    keySignalStatus = 1;
  }
  else{
    keyRestarting = 2;
  }
}

void rechargeOn(void)
{
  DQchargerEnable = 1;
  ResetCycleTimer(CT_RECHARGE_DELAY);
  if (battType == CUMMINS_REV1){
    ResetCycleTimer(CT_CUMMINS_CHRG_DELAY);
    digitalWrite(CHARGE_EN, HIGH);
    rechargeState = 1;
  }
  else if (battType == INVNTS_OLD){
    digitalWrite(CHARGE_EN, HIGH);
    digitalWrite(INVNTS_DISCHRG, LOW);
    rechargeState = 1;
  }
  else if (battType == INVNTS_60AH){
    digitalWrite(CHARGE_EN, HIGH);
    digitalWrite(INVNTS_DISCHRG, LOW);
    rechargeState = 1;
  }
  else if (battType == INVNTS_80AH){
    digitalWrite(CHARGE_EN, HIGH);
    digitalWrite(INVNTS_DISCHRG, LOW);
    rechargeState = 1;
  }
  else if (battType == INVNTS_VIRT_BATT){
    digitalWrite(CHARGE_EN, HIGH);
    digitalWrite(INVNTS_DISCHRG, HIGH);
    keySignalStatus = 1;
    rechargeState = 1;
  }
  else if (moduleMinTemperature > CHARGE_MIN_TEMP){
    digitalWrite(CHARGE_EN, HIGH);
    digitalWrite(INVNTS_DISCHRG, HIGH);
    rechargeState = 1;
  }
  else{
    rechargeOff();
  }
}

void rechargeOff(void)
{
  if (battType == INVNTS_OLD){
    digitalWrite(INVNTS_DISCHRG, HIGH);
  }
  if (battType == INVNTS_60AH){
    digitalWrite(INVNTS_DISCHRG, HIGH);
  }
  if (battType == INVNTS_80AH){
    digitalWrite(INVNTS_DISCHRG, HIGH);
  }
  if (battType == INVNTS_VIRT_BATT){
    digitalWrite(INVNTS_DISCHRG, HIGH);
    keySignalStatus = 1;
  }
  digitalWrite(CHARGE_EN, LOW);  //low
  rechargeState = 0;
}

void ResetCycleTimer(CYCLE_TIMERS eCycleTimer)
{
  if (eCycleTimer < NUM_CYCLE_TIMERS)
  {
    if (eCycleTimer == CT_LOG_INTERVAL){
      SetTimerWithMilliseconds(&scastCycleTimers[eCycleTimer], (unsigned int)(sdLogRate*1000));
    }
    else{
      SetTimerWithMilliseconds(&scastCycleTimers[eCycleTimer], scawCycleTestTimers[eCycleTimer]);
    }
  }
}

int fSetChrgCurr(String setCurrString){
  fixedOverRideCurr = setCurrString.toFloat();
  //fixedOverRideCurr = setDQCurrent;
  DQfixedCurrOverride = 1;
  if (fixedOverRideCurr < 0) {
    DQfixedCurrOverride = 0;
  }
  return 1;
}

int fSetChrgVolt(String setVoltString){
  fixedOverRideVolt = setVoltString.toFloat();
  DQfixedVoltOverride = 1;
  if (fixedOverRideVolt < 0){
    DQfixedVoltOverride = 0;
  }
  return 1;
}

int fStateTest(String command)
{
    if (command == "halt"){
        testStatePrev = testState;
        testState = stateCOMMANDHALT;
        LogUserString("#HALT_COMMANDED\n");
        return 1;
    }
    if (command == "init"){
        testState = stateINIT;
        LogUserString("#INIT_COMMANDED\n");
        return 1;
    }
    if (command == "discharge"){
        testStatePrev = testState;
        testState = statePOWERRESETCHRGOFF;
        LogUserString("#DISCHARGE_COMMANDED\n");
        return 1;
    }
    if (command == "charge"){
        testStatePrev = testState;
        testState = stateDISCHARGE_INPUTOFF;
        LogUserString("#CHARGE_COMMANDED\n");
        return 1;
    }
    if (command == "holdcharge"){
        testStatePrev = testState;
        testState = stateDISCHARGE_INPUTOFF;
        LogUserString("#HOLD_CHARGE_COMMANDED\n");
        holdCharge = 1;
        return 1;
    }
    if (command == "pause")  //pause only prevents CANbus error induced HALT and stops the recording of data to the SD Card
    {
        if (flagSDPause == 0)
        {
            flagSDPause = 1;
            testStatePrev = testState;
            testState = statePAUSE;
            LogUserString("#PAUSED\n");
        }
        else
        {
            flagSDPause = 0;
            testState = testStatePrev;
            LogUserString("#UNPAUSED\n");
        }
        // close the file:

        return 1;
    }

    else return -1;
}

int fSetMinDischargePercent(String minDischargePrcnt){
  minDischargePercent = minDischargePrcnt.toInt();
  return 1;
}

int fSetMaxChargePercent(String maxChargePrcnt){
  maxChargePercent = maxChargePrcnt.toInt();
  return 1;
}

int okToDischarge(void){
  static int contactorStatFlag = 0;

  if (underTempDischargeStatus > ALARM){
    return 2;
  }
  else if ((overTemperatureStatus > ALARM) && (overTempDischargeStatus > ALARM)){
    return 3;
  }
  else if ((PCBAoverTempStatus > ALARM) && (shortCircuitStatus > ALARM)){
    return 4;
  }
  else if ((battTerminalOverTempStatus > ALARM) && (otherDischargeFaultStatus > ALARM)){
    return 5;
  }
  //else if (overVoltageStatus > ALARM){
    //return 6;                               //over voltage is not a problem for discharging
  //}
  else if (underVoltageStatus >= ALARM){
    return 7;
  }
  else if (overCurrentDischarge > ALARM){
    return 8;
  }
  else if ((cellDeltaTempStatus > ALARM) && (internalCommStatus > ALARM)){
    return 9;
  }
  else if (ModuleLostState > ALARM){
    return 10;
  }
  else if ((BMSrev3CANok() == 0)&&(InvntsOldCANok() == 0)&&(Invnts60AhCANok() == 0)&&(Invnts80AhCANok() == 0)&&(InvntsVirtualBattCANok() == 0)){
    return 11;
  }
  else if (moduleMinTemperature <= DISCHARGE_MIN_TEMP){
    return 12;
  }
  else if (moduleMaxTemperature > MODULE_MAX_TEMP){
    return 13;
  }
  else if ((battType == VALENCE_REV3)&&(contactorMain == 0)){
    if (contactorStatFlag == 0){
      ResetCycleTimer(CT_CNTCTR_DELAY);
      contactorStatFlag = 1;
      return 0;
    }
    else if (TimerExpired(&scastCycleTimers[CT_CNTCTR_DELAY])){
      return 14;
    }
  }
  else if (((battType == INVNTS_OLD) || (battType == INVNTS_60AH) || (battType == INVNTS_80AH)||(battType == INVNTS_VIRT_BATT))&&(maxDischargeCurrent == 0)){
    return 15;
  }
  else if (((battType == INVNTS_OLD) || (battType == INVNTS_60AH) || (battType == INVNTS_80AH)||(battType == INVNTS_VIRT_BATT))&&(moduleSOCscale <= minDischargePercent)){
    return 16;
  }
  else{
    contactorStatFlag = 0;
    return 1;
  }
  return -1;
}

int okToCharge(void){
  if (underTempChargeStatus > ALARM){
    if (battType == INVNTS_OLD){
      if (BMSchargeCurrSetpoint > 0){
        BMSchargeVoltSetpoint = 30.4;
      }
    }
    else if (battType == INVNTS_60AH){
      if (BMSchargeCurrSetpoint > 0){
        BMSchargeVoltSetpoint = 30.4;
      }
    }
    else if (battType == INVNTS_80AH){
      if ((BMSchargeCurrSetpoint > 0)|| (InvntsHeaterStat==1)){ //BMSchargeCurrSetpoint > 0
        BMSchargeVoltSetpoint = 30.4;
      }
    }
    else if (battType == INVNTS_VIRT_BATT){
      if ((BMSchargeCurrSetpoint > 0)|| (InvntsHeaterStat==1)){ //BMSchargeCurrSetpoint > 0
        //BMSchargeVoltSetpoint = 30.4;
      }
    }
    return 2;
  }
  else if ((overTemperatureStatus > ALARM) && (overTempChargeStatus > ALARM)){
    return 3;
  }
  else if ((PCBAoverTempStatus > ALARM) && (shortCircuitStatus > ALARM)){
    return 4;
  }
  else if ((battTerminalOverTempStatus > ALARM) && (otherChargeFaultStatus > ALARM)){
    return 5;
  }
  else if (overVoltageStatus > ALARM){
    return 6;
  }
  else if (overCurrentCharge > ALARM){
    return 7;
  }
  else if ((cellDeltaTempStatus > ALARM) && (internalCommStatus > ALARM)){
    return 8;
  }
  else if (ModuleLostState > ALARM){
    return 9;
  }
  else if ((BMSrev3CANok() != 1)||(InvntsOldCANok() != 1)||(Invnts60AhCANok() != 1)||(Invnts80AhCANok() != 1)||(InvntsVirtualBattCANok() != 1)){
    return 10;
  }
  else if (moduleMinTemperature < CHARGE_MIN_TEMP){
    return 11;
  }
  else if (moduleMaxTemperature > MODULE_MAX_TEMP){
    return 12;
  }
  else{
    if (battType == INVNTS_OLD){
      if (BMSchargeCurrSetpoint > 0){
        BMSchargeVoltSetpoint = 30.3;
      }
    }
    else if (battType == INVNTS_60AH){
      if (BMSchargeCurrSetpoint > 0){
        BMSchargeVoltSetpoint = 30.3;
      }
    }
    else if (battType == INVNTS_80AH){
      if ((BMSchargeCurrSetpoint > 0) || (InvntsHeaterStat==1)){  //BMSchargeCurrSetpoint > 0
        BMSchargeVoltSetpoint = 30.3;
      }
    }
    else if (battType == INVNTS_VIRT_BATT){
      if ((BMSchargeCurrSetpoint > 0) || (InvntsHeaterStat==1)){  //BMSchargeCurrSetpoint > 0
        //BMSchargeVoltSetpoint = 30.4;
      }
    }
    return 1;
  }
}

float maxCSMSurfaceTemp(void){
  float maxSurfTemp = -99;
  for (int i = 0; i<3; i++){
    if (CsmTemps[i]>maxSurfTemp){
      maxSurfTemp = CsmTemps[i];
    }
  }
  return maxSurfTemp;
}


void turnHeaterOn(void){
  if (battType == VALENCE_REV3){
    if(maxSurfTempReturned < HEATER_ON_SURFACE_TEMP){
      //setDQCurrent = 6;
      digitalWrite(HEATER, HIGH);
      heaterStatus = 1;
    }
    else{
      digitalWrite(HEATER, LOW);
      //setDQCurrent = prevDQCurr;
      heaterStatus = 0;
    }
  }
  else if (battType == INVNTS_OLD){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 1);
    //heaterStatus = 1;
  }
  else if (battType == INVNTS_60AH){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 1);
    //heaterStatus = 1;
  }
  else if (battType == INVNTS_80AH){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 1);
    //heaterStatus = 1;
  }
  else if (battType == INVNTS_VIRT_BATT){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 1);
    //heaterStatus = 1;
  }
}

void turnHeaterOff(void){
  if (battType == VALENCE_REV3){
    digitalWrite(HEATER, LOW);
    heaterStatus = 0;
  }
  if (battType == INVNTS_OLD){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 0);
    //heaterStatus = 0;
  }
  if (battType == INVNTS_60AH){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 0);
    //heaterStatus = 0;
  }
  if (battType == INVNTS_80AH){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 0);
    //heaterStatus = 0;
  }
  if (battType == INVNTS_VIRT_BATT){
    //InvntsSDOWriteReq(INVNTS_HEATER_CONTROL_SUBINDEX, 0);
    //heaterStatus = 0;
  }
}

void manageHeaterEnabled(void){
  if (battType == VALENCE_REV3){
    if(maxSurfTempReturned < HEATER_ON_SURFACE_TEMP){
      turnHeaterOn();
    }
    else if (maxSurfTempReturned >= HEATER_OFF_SURFACE_TEMP){
      turnHeaterOff();
    }
  }
  else if (battType == INVNTS_OLD){
    if (moduleMinTemperature <= CHRGR_PWR_HEATER_ON_CELL_TEMP){
      turnHeaterOn();
    }
    else if (moduleMinTemperature > CHRGR_PWR_HEATER_OFF_CELL_TEMP){
      turnHeaterOff();
    }
  }
  else if (battType == INVNTS_60AH){
    if (moduleMinTemperature <= CHRGR_PWR_HEATER_ON_CELL_TEMP){
      turnHeaterOn();
    }
    else if (moduleMinTemperature > CHRGR_PWR_HEATER_OFF_CELL_TEMP){
      turnHeaterOff();
    }
  }
  else if (battType == INVNTS_80AH){
    if (moduleMinTemperature <= CHRGR_PWR_HEATER_ON_CELL_TEMP){
      turnHeaterOn();
    }
    else if (moduleMinTemperature > CHRGR_PWR_HEATER_OFF_CELL_TEMP){
      turnHeaterOff();
    }
  }
  else if (battType == INVNTS_VIRT_BATT){
    if (moduleMinTemperature <= CHRGR_PWR_HEATER_ON_CELL_TEMP){
      turnHeaterOn();
    }
    else if (moduleMinTemperature > CHRGR_PWR_HEATER_OFF_CELL_TEMP){
      turnHeaterOff();
    }
  }
}

int fStateHeater(String heaterCommand){
  if (heaterCommand == "force"){
    //turnHeaterOn();
    //heaterCommandOverride = 1;
    return -1;
  }
  else if (heaterCommand == "on"){
    heaterCommandOverride = 1;
    heaterState = heaterForceOn;
    return 1;
  }
  else if (heaterCommand == "off"){
    heaterCommandOverride = 1;
    heaterState = heaterOFF;
    return 1;
  }
  else if (heaterCommand == "auto"){
    heaterCommandOverride = 0;
    heaterState = heaterOFF;
    return 1;
  }
  else{
    return -1;
  }
}

void manageSDLogging(void){
  if ((flagSDPause == 0) && (TimerExpired(&scastCycleTimers[CT_LOG_INTERVAL]))){
    Log2SD(what2logTxt);
    ResetCycleTimer(CT_LOG_INTERVAL);
  }
}
