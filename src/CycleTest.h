#include "application.h"

const int CHARGE_EN = D3;  //enables charging and opens discharge contactor
const int IGN = D4;       //enables the ignition line on BMS (used for reseting alarm(s) after discharge cycle)
const int HEATER = D5;  //enables the heater relay
const int BRAMMO_INTRLK = D5;   //enables dischange of the Cummins-Brammo Battery

typedef enum
{
  CT_KEY_RESTART_DELAY,
  CT_KEY_WAKE,
  CT_EL_LOAD_CMD_DELAY,
  CT_CNTCTR_DELAY,
  CT_RECHARGE_DELAY,
  CT_CUMMINS_CHRG_DELAY,
  CT_LOG_INTERVAL,
  NUM_CYCLE_TIMERS,
  FIRST_CYCLE_TIMER = 0
} CYCLE_TIMERS;

void initCycleTest(void);
void CycleTest(void);

enum stateTestControl
{
    stateINIT,                  //0
    statePOWER_ON,              //1
    statePWRRESET_LOAD_OFF,     //2
    statePOWERRESETCHRGON,      //3
    statePOWERRESET_CHARGE,    //4
    statePOWERRESETCHRGOFF,     //5
    stateSTARTDISCHARGE,        //6
    stateDISCHARGE,             //7
    stateDISCHARGE_INPUTOFF,    //8
    stateCHARGEENABLE,          //9
    stateCHARGE,                //10
    stateENDOFCHRG,             //11
    stateREPORTOUT,             //12
    stateCOMMANDHALT,           //13
    stateERRORHALT,             //14
    stateHALT,                  //15
    statePAUSE,                 //16
    stateKEYRESTART,            //17
    stateNumberOfStates
};

enum heaterStateControl
{
  heaterOFF,
  heaterForceOn,
  heaterONnotCharging,
  heaterON_Charging,
  heaterError,
  heaterDisabled,
  heaterNumOfStates
};

static const String stateStrings[stateNumberOfStates] =
{
  "stateINIT",                  //0
  "statePOWER_ON",              //1
  "statePWRRESET_LOAD_OFF",     //2
  "statePOWERRESETCHRGON",      //3
  "statePOWERRESET_CHARGE",    //4
  "statePOWERRESETCHRGOFF",     //5
  "stateSTARTDISCHARGE",        //6
  "stateDISCHARGE",             //7
  "stateDISCHARGE_INPUTOFF",    //8
  "stateCHARGEENABLE",          //9
  "stateCHARGE",                //10
  "stateENDOFCHRG",             //11
  "stateREPORTOUT",             //12
  "stateCOMMANDHALT",           //13
  "stateERRORHALT",             //14
  "stateHALT",                  //15
  "statePAUSE",                 //16
  "stateKEYRESTART",            //17
};

static const String heaterStateStrings[heaterNumOfStates] =
{
  "heaterOFF",                //0
  "heaterForceOn",            //1
  "heaterONnotCharging",      //2
  "heaterON_Charging",        //3
  "heaterError",              //4
  "heaterDisabled",           //5
};
