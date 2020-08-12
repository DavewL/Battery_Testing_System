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

static const unsigned int scawCycleTestTimers[NUM_CYCLE_TIMERS] =
{
  /* CT_KEY_RESTART_DELAY   */   2000,  //ms -TIME TO KEEP THE IGNITION SIGNAL off
  /* CT_KEY_WAKE            */   9000,  //ms -TIME TO WAIT AFTER KEY ON SIGNAL
  /* CT_EL_LOAD_CMD_DELAY   */   200,   //ms -TIME TO WAIT BETWEEN COMMANDS TO THE EL. LOAD
  /* CT_MAIN_CNTCTR_DELAY   */   2000,  //ms -TIME LAG OF MAIN CONTACTOR STATE REPORTING ON CANBUS
  /* CT_RECHARGE_DELAY      */   8000, //ms -MAX TIME TO WAIT AFTER POWERING ON CHARGER -CUMMINS ONLY
  /* CT_CUMMINS_CHRG_DELAY  */   6000,  //ms -MIN TIME TO WAIT AFTER POWERING ON CHARGER -CUMMINS ONLY -MUST BE SHORTER THAN CT_RECHARGE_DELAY
  /* CT_LOG_INTERVAL        */   4000,   //ms -TIME TO WAIT BETWEEN COMMANDS TO THE EL. LOAD
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
String heaterState2String = "heaterOFF";
String heaterStatus2String;
String cellsDeltaV2String;

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
int okToDischarge(void);
int okToCharge(void);
void turnHeaterOn(void);
void turnHeaterOff(void);
void manageHeaterEnabled(void);
float maxCSMSurfaceTemp(void);
void manageSDLogging(void);

int fElLoadString(String command){
  Serial1.println(command);
  Serial.println(command);
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
  Particle.variable("OkToCharge", ok2ChargeStatus);
  Particle.variable("HeaterState", heaterState2String);
  Particle.variable("HeaterStatus", heaterStatus2String);
  Particle.variable("StateMachine", testState2String);
  Particle.variable("BattVoltage", battVolt2String);
  Particle.variable("BattCurrent", battCurr2String);
  Particle.variable("SOC", battSOC2String);
  Particle.variable("CellMinTemp", battMinTemp2String);
  Particle.variable("CellMaxTemp", battMaxTemp2String);
  Particle.variable("CellDeltaV", cellsDeltaV2String);

  pinMode(CHARGE_EN, OUTPUT);
  pinMode(IGN, OUTPUT);
  if (battType == VALENCE_REV3){
    pinMode(HEATER, OUTPUT);
  }
  else{
    pinMode(BRAMMO_INTRLK, OUTPUT);
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
  cellsDeltaV2String = String::format("%d",cellsDeltaV);
  battMaxTemp2String = String::format("%d", moduleMaxTemperature);
  battMinTemp2String = String::format("%d", moduleMinTemperature);
  cellsDeltaV = moduleMaxMvolts - moduleMinMvolts;
  battSOC2String = String::format("%.2f", moduleSOCscale);
  battVolt2String = String::format("%.3f", battVoltage);
  battCurr2String = String::format("%.1f", battCurrent);

  int DQcurrentSetpoint = 0;
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

  int tempOk2ChargeStatus = okToCharge();
  ok2ChargeStatus = String::format("%d",tempOk2ChargeStatus);

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
      initSerialElLoad();
      if (readFrameFileExists == 1){
        LogUserString("#CYCLES,#PH_CYCLES,#RECHARGE_CYCLES");
      }
      break;

    case statePOWER_ON:          //1
      //transmitDQGoOp();
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
      testState = statePOWERRESETCHRGON;
      break;

    case statePOWERRESETCHRGON:   //3
      IC1200Enable = 0;
      setIC1200Current = 0;
      setIC1200Voltage = 0;

      if (battType == VALENCE_REV3){
        if (BMSrev3CANok()){
          if (moduleSOCscale < 100.0){
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
          //if ((DQwallPluggedIn == 1)||(TimerExpired(&scastCycleTimers[CT_RECHARGE_DELAY]))){
          if (TimerRunning(&scastCycleTimers[CT_RECHARGE_DELAY])){
            if (DQwallPluggedIn == 1){
              if (moduleSOCscale < 100.0){
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
      /*  if (1){  //CumminsCANok()
          if (moduleSOCscale < 100.0){//(DQwallPluggedIn == 1){
            if //(DQwallPluggedIn == 1){
              rechargeOn();
              testState = statePOWERRESET_CHARGE;  //start charging
            }
            else ()
          }
          else{
            testState = statePOWERRESETCHRGOFF;
          }*/
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
      break;

    case statePOWERRESETCHRGOFF:  //5
      flagSDPause = 1;
      rechargeOff();
      testState = stateSTARTDISCHARGE;
      break;

    case stateSTARTDISCHARGE:     //6
      cumlAmpHrs = 0;
      cumlWattHrs = 0;
      if (battType == CUMMINS_REV1){
        digitalWrite(BRAMMO_INTRLK, HIGH);
      }
      if (dischargerType == SD_REPLAY){
        //configure discharger for replicating list read from SD card
        RestartFrameFileLines();
        testSubCycleCount = 0;
        setElLoad2PwrMode();
        ReadFrameLine();  //begin reading data frames from the SD card
        testState = stateDISCHARGE;
      }
      else {
        if (startListRunning()){  //when function returns a 1, all messages have been sent
          testState = stateDISCHARGE;
        }
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
        //if (cumminsDischrgEnabled == 0){
          //testState = stateDISCHARGE_INPUTOFF;
        //}
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
      break;

    case stateCHARGEENABLE:       //9
      IC1200Enable = 0;
      setIC1200Current = 0;
      setIC1200Voltage = 0;

      if(battType == VALENCE_REV3){
        if (BMSrev3CANok()){
          if (moduleSOCscale < 100.0){
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
      break;

    case  stateENDOFCHRG:             //11
      rechargeOff();
      //flagSDPause = 1;
      testState = stateREPORTOUT;
      break;

    case  stateREPORTOUT:             //12
      cumlAmpHrs = 0;
      cumlWattHrs = 0;
      testCycleCount++;
      testState = stateSTARTDISCHARGE;
      break;

    case  stateCOMMANDHALT:           //13
      testState = stateHALT;
      break;

    case  stateERRORHALT:             //14
      testState = stateHALT;
      break;

    case  stateHALT:                  //15
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

  if (heaterStatus == 1){
    DQcurrentSetpoint += 7;
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
  if (1){  //set dq current setpoint based on BMS current request
    DQcurrentSetpoint = DQcurrentSetpoint + BMSchargeCurrSetpoint;
  }
  if ((moduleMaxTemperature > MODULE_HIGH_TEMP_CHRG_CUTBACK_ON)&&(DQcurrentSetpoint > 15)){
    DQcurrentSetpoint = 15;
  }

  setDQCurrent = DQcurrentSetpoint;
  setDQVoltage = BMSchargeVoltSetpoint;

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
  ResetCycleTimer(CT_RECHARGE_DELAY);
  if (battType == CUMMINS_REV1){
    ResetCycleTimer(CT_CUMMINS_CHRG_DELAY);
    digitalWrite(CHARGE_EN, HIGH);
    rechargeState = 1;
  }
  else if (moduleMinTemperature > CHARGE_MIN_TEMP){
    digitalWrite(CHARGE_EN, HIGH);
    rechargeState = 1;
  }
  else{
    rechargeOff();
  }
}

void rechargeOff(void)
{
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
  return 1;
}

int fSetChrgVolt(String setVoltString){
  fixedOverRideVolt = setVoltString.toFloat();
  DQfixedVoltOverride = 1;
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

int okToDischarge(void){
  static int contactorStatFlag = 0;

  if (underTempDischargeStatus > ALARM){
    return 2;
  }
  else if (overTemperatureStatus > ALARM){
    return 3;
  }
  else if (PCBAoverTempStatus > ALARM){
    return 4;
  }
  else if (battTerminalOverTempStatus > ALARM){
    return 5;
  }
  else if (overVoltageStatus > ALARM){
    return 6;
  }
  else if (underVoltageStatus >= ALARM){
    return 7;
  }
  else if (overCurrentDischarge > ALARM){
    return 8;
  }
  else if (cellDeltaTempStatus > ALARM){
    return 9;
  }
  else if (ModuleLostState > ALARM){
    return 10;
  }
  else if (BMSrev3CANok() != 1){
    return 11;
  }
  else if (moduleMinTemperature <= DISCHARGE_MIN_TEMP){
    return 12;
  }
  else if (moduleMaxTemperature > MODULE_MAX_TEMP){
    return 13;
  }
  else if (contactorMain == 0){
    if (contactorStatFlag == 0){
      ResetCycleTimer(CT_CNTCTR_DELAY);
      contactorStatFlag = 1;
    }
    else if (TimerExpired(&scastCycleTimers[CT_CNTCTR_DELAY])){
      return 14;
    }
  }
  //else if (maxDischargeCurrent <= 1){
    //return 12;
  //}
  else{
    contactorStatFlag = 0;
    return 1;
  }
}

int okToCharge(void){
  if (underTempChargeStatus > ALARM){
    return 2;
  }
  else if (overTemperatureStatus > ALARM){
    return 3;
  }
  else if (PCBAoverTempStatus > ALARM){
    return 4;
  }
  else if (battTerminalOverTempStatus > ALARM){
    return 5;
  }
  else if (overVoltageStatus > ALARM){
    return 6;
  }
  else if (overCurrentCharge > ALARM){
    return 7;
  }
  else if (cellDeltaTempStatus > ALARM){
    return 8;
  }
  else if (ModuleLostState > ALARM){
    return 9;
  }
  else if (BMSrev3CANok() != 1){
    return 10;
  }
  else if (moduleMinTemperature <= CHARGE_MIN_TEMP){
    return 11;
  }
  else if (moduleMaxTemperature > MODULE_MAX_TEMP){
    return 12;
  }
  //else if (contactorCharge == 0){
  //  return 13;
  //}
  //else if (maxRegenCurrent <= 1){
  //  return 11;
  //}
  else{
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
}

void turnHeaterOff(void){
  if (battType == VALENCE_REV3){
    digitalWrite(HEATER, LOW);
    heaterStatus = 0;
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
