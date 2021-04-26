#include "Display.h"
#include "Log2SdCard.h"
#include <LiquidCrystal_I2C_Spark.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_mfGFX.h>
#include "Globals.h"
#include "defines.h"
#include "Rev3Messages.h"
#include "CumminsCAN.h"
#include "InvntsOldCAN.h"
#include "Invnts60AhCAN.h"
#include "Invnts80AhCAN.h"

extern ElLoadFrame CurrLoadFrame;

LiquidCrystal_I2C *lcd;

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void initDisplay(int dispType){
  if (dispType == FOUR_LINE_LCD){
      //setup LCD display
      lcd = new LiquidCrystal_I2C(0x27,20,4);
      lcd->init();
      lcd->backlight();
      lcd->clear();
      lcd->setCursor(0,0);
      lcd->print("***Battery Status***");
  }

  if (dispType == THREEPOINTTWO_INCH_NEXTION){

  }

  if (dispType == TWOFORTY_BY_THREETWENTY_SERIAL){
    tft.begin();
    /*
    // read diagnostics (optional but can help debug problems)
    uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
    x = tft.readcommand8(ILI9341_RDMADCTL);
    */
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.fillScreen(ILI9341_BLACK);
    //tft.setCursor(0, 0); tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
    //tft.println("DaVinci System");
  }

}

void dispBatteryHeading(){
  String tempSDstatus = "   SD CARD FAILED";
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(XCANSTAT, 0);
  if (SDcardInitOK){
    tempSDstatus = "  SD CARD LOGGING";
    tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
  }
  else { tft.setTextColor(ILI9341_BLACK,ILI9341_RED); }
  tft.setTextSize(1);
  tft.print(tempSDstatus);
  tft.setCursor(0, YBATTHEADING); tft.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);  tft.setTextSize(3);
  tft.print("BATTERY INFO");
  tft.setCursor(0, YSOC); tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);  tft.setTextSize(2);
  tft.print("CHARGE REMAINING:");
  tft.setCursor(0,YCURRENT);
  tft.print("BATTERY CURRENT:");
  tft.setCursor(0,YMINTEMP);
  tft.print("BATT MIN TEMP:");
  tft.setCursor(0,YMAXTEMP);
  tft.print("BATT MAX TEMP:");
  tft.setCursor(0,YVOLTAGE);
  tft.print("VOLTAGE:");
  tft.setCursor(0,YCSMTEMPS);
  tft.print("CSM:");
  tft.setCursor(0,YCHRGSTAT);
  tft.print("CHARGE STATUS:");
  if(readFrameFileExists==1){
    tft.setCursor(0,YSDREADSTAT);
    tft.print("SD READ:");
  }
}

void dispBatteryStats(){
  String tempSpaces = "";
  String tempCANstatus = "      CANBUS LOST";
  String SSIDname = WiFi.SSID();
  static int firstRun = 1;
  static int CANok = 0;
  static int prevCANok =0;
  static int prevChrgStat =0;
  static int WifiOk = 0;
  static int prevWiFiOk = 0;
  static int prevCloudOk = 0;

  if (battType==VALENCE_REV3){
    if (BMSrev3CANok()){
      CANok = 1;
    }
    else{
      CANok = 0;
    }
  }
  else if (battType == CUMMINS_REV1){
    if (CumminsCANok()){
      CANok = 1;
    }
    else{
      CANok = 0;
    }
  }
  else if (battType == INVNTS_OLD){
    if (InvntsOldCANok()){
      CANok = 1;
    }
    else{
      CANok = 0;
    }
  }
  else if (battType == INVNTS_60AH){
    if (Invnts60AhCANok()){
      CANok = 1;
    }
    else{
      CANok = 0;
    }
  }
  else if (battType == INVNTS_80AH){
    if (Invnts80AhCANok()){
      CANok = 1;
    }
    else{
      CANok = 0;
    }
  }

  if ((firstRun == 1)||(CANok != prevCANok)){
    tft.setCursor(XCANSTAT, YCANSTAT);
    tft.setTextColor(ILI9341_BLACK,ILI9341_RED);
    tft.setTextSize(1);
    if(CANok == 1){
      tempCANstatus = "           CAN OK";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
    tft.print(tempCANstatus);
  }
  prevCANok = CANok;

  //---------------------------------CHARGE STATUS--------------------------------
  if ((firstRun == 1)||(chargeStatus != prevChrgStat)){
    tempSpaces = "";
    tft.setCursor(XCHRGSTAT, YCHRGSTAT); tft.setTextSize(2);
    if (chargeStatus == 0){
      tempSpaces.concat("        MAIN");
    }
    else if (chargeStatus == 1){
      tempSpaces.concat("EQUALIZATION");
    }
    else if (chargeStatus == 2){
      tempSpaces.concat("       FLOAT");
    }
    else if (chargeStatus == 3){
      tempSpaces.concat("          NA");
    }
    else if (chargeStatus == 4){
      tempSpaces.concat("    COMPLETE");
    }
    tft.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);
    tft.print(tempSpaces);
    //--------------------------------/CHARGE STATUS/-------------------------------
  }
  prevChrgStat = chargeStatus;

  //-------------------------------WIFI STATUS------------------------------------
  if (SSIDname != ""){
    WifiOk = 1;
  }
  else{
    WifiOk = 0;
  }
  if ((firstRun == 1)||(WifiOk != prevWiFiOk)){
    tempSpaces = " NO WIFI ";
    tft.setTextColor(ILI9341_BLACK,ILI9341_RED);
    tft.setCursor(XWIFISTAT, YWIFISTAT); tft.setTextSize(1);
    if (SSIDname != ""){
      tempSpaces = SSIDname;
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
    tft.print(tempSpaces);
    //------------------------------/WIFI STATUS/-----------------------------------
  }
  prevWiFiOk = WifiOk;

  //----------------------------SPARK CLOUD STATUS--------------------------------
  if ((firstRun == 1)||(SparkNetOk != prevCloudOk)){
    tempSpaces = "NO INTERNET    ";
    tft.setTextColor(ILI9341_BLACK,ILI9341_RED);
    tft.setCursor(XNETSTAT, YNETSTAT); tft.setTextSize(1);
    if (SparkNetOk == 1){
      tempSpaces = "CLOUD CONNECTED";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
    tft.print(tempSpaces);
    //--------------------------/SPARK CLOUD STATUS/--------------------------------
  }
  prevCloudOk = SparkNetOk;

  firstRun = 0;
}

void dispBatterySOC(void){
  //-------------------------------------SOC--------------------------------------
  String tempSpaces = "";
  String tempString;
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  if (moduleSOCscale < 100.0){
    if (moduleSOCscale < 10.0){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  tempString = String::format("%.1f",moduleSOCscale);
  tft.setCursor(XSOC, YSOC); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat("%");
  tft.print(tempSpaces);
//------------------------------------/SOC/-------------------------------------
}

void dispBatteryVolt(void){
  //----------------------------------VOLTAGE-------------------------------------
  String tempSpaces = "";
  String tempString;

  if (battVoltage < 100.0){
    if (battVoltage < 10.0){
      tempSpaces = " ";
    }
    else{
    tempSpaces = "";
    }
  }
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  tempString = String::format("%.2f",battVoltage);
  tft.setCursor(XVOLTAGE, YVOLTAGE); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat("V");
  tft.print(tempSpaces);

  //--------------------------------/VOTLAGE/-------------------------------------
}

void dispBatteryAmps(void){
  //-----------------------------------CURRENT------------------------------------
  String tempSpaces = "";
  String tempString;

  if (abs(battCurrent) < 100){
    if (abs(battCurrent) < 10){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (battCurrent >= 0){
    tempSpaces.concat(" ");
  }
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  tempString = String::format("%.1f",battCurrent);
  tft.setCursor(XCURRENT, YCURRENT); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat("A");
  tft.print(tempSpaces);
  //-----------------------------------/CURRENT/----------------------------------
}

void dispBatteryTemps(void){
  //--------------------------------MIN TEMPERATURE-------------------------------
  String tempSpaces = "";
  String tempString;

  if (abs(moduleMinTemperature) < 100){
    if (abs(moduleMinTemperature) < 10){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (moduleMinTemperature >= 0){
    tempSpaces.concat(" ");
  }
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
  tempString = String::format("%.0f",moduleMinTemperature);
  tft.setCursor(XMINTEMP, YMINTEMP); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat(" C");
  tft.print(tempSpaces);
  //------------------------------/MIN TEMPERATURE/-------------------------------

  //-------------------------------MAX TEMPERATURE--------------------------------
  tempSpaces = "";
  if (abs(moduleMaxTemperature) < 100){
    if (abs(moduleMaxTemperature) < 10){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (moduleMaxTemperature >= 0){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%.0f",moduleMaxTemperature);
  tft.setCursor(XMAXTEMP, YMAXTEMP); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat(" C");
  tft.print(tempSpaces);
  //-----------------------------/MAX TEMPERATURE/--------------------------------
}

void dispBatteryCSM(void){
  //-----------------------------CSM TEMPERATURES---------------------------------
  String tempSpaces = "";
  String tempString;
  String tempLine = "";
  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);

  if (abs(CsmTemps[0]) < 1000){
    if (abs(CsmTemps[0]) < 100){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (CsmTemps[0] > -10){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%d",CsmTemps[0]/10);
  tft.setCursor(XCSMTEMPS, YCSMTEMPS); tft.setTextSize(3);
  tempSpaces.concat(tempString);
  tempLine.concat(tempSpaces);

  tempSpaces = "";
  if (abs(CsmTemps[1]) < 1000){
    if (abs(CsmTemps[1]) < 100){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (CsmTemps[1] > -10){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%d",CsmTemps[1]/10);
  tempSpaces.concat(tempString);
  tempLine.concat(tempSpaces);

  tempSpaces = "";
  if (abs(CsmTemps[2]) < 1000){
    if (abs(CsmTemps[2]) < 100){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (CsmTemps[2] > -10){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%d",CsmTemps[2]/10);
  tempSpaces.concat(tempString);
  tempLine.concat(tempSpaces);

  tempLine.concat("C");
  tft.print(tempLine);
  //-------------------------------/CSM TEMPERATURES/-----------------------------
}

void dispBatterySDPwr(void){
  //------------------------------SD READ STATUS----------------------------------
  String tempSpaces = "";
  String tempString;
  if(readFrameFileExists==1){
    if (abs(CurrLoadFrame.Power) < 10000){
      if (abs(CurrLoadFrame.Power) < 1000){
        if (abs(CurrLoadFrame.Power) < 100){
          if (abs(CurrLoadFrame.Power) < 10){
            if (CurrLoadFrame.Power == 0){
              tempSpaces = "     ";
            }
            else{
            tempSpaces = "    ";
            }
          }
          else{
          tempSpaces = "   ";
          }
        }
        else{
        tempSpaces = "  ";
        }
      }
      else{
      tempSpaces = " ";
      }
    }
    if (CurrLoadFrame.Power > 0){
      tempSpaces.concat(" ");
    }
    //tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
    tft.setCursor(XSDREADSTAT, YSDREADSTAT); tft.setTextSize(2);
    tempString = String::format("%.0f",CurrLoadFrame.Power);
    tempSpaces.concat(tempString);
    tempSpaces.concat(" W");
    tft.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);
    tft.print(tempSpaces);
  }
  //-----------------------------/SD READ STATUS/---------------------------------
}

void dispBattery(){
  static int counts = 0;
  String tempSpaces = "";
  String tempString;
  String tempSDstatus = "   SD CARD FAILED";
  String tempCANstatus = "      CANBUS LOST";
  String tempLine = "";

  String SSIDname = WiFi.SSID();
  //tft.fillRect(234,28, 176, 24, ILI9341_BLACK);
  //tft.setCursor(0, 0); tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);  tft.setTextSize(2);
  if (counts == 0){
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(XCANSTAT, 0);
    if (SDcardInitOK){
      tempSDstatus = "  SD CARD LOGGING";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
    else { tft.setTextColor(ILI9341_BLACK,ILI9341_RED); }
    tft.setTextSize(1);
    tft.print(tempSDstatus);
    tft.setCursor(0, YBATTHEADING); tft.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);  tft.setTextSize(3);
    tft.print("BATTERY INFO");
    tft.setCursor(0, YSOC); tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);  tft.setTextSize(2);
    tft.print("CHARGE REMAINING:");
    tft.setCursor(0,YCURRENT);
    tft.print("BATTERY CURRENT:");
    tft.setCursor(0,YMINTEMP);
    tft.print("BATT MIN TEMP:");
    tft.setCursor(0,YMAXTEMP);
    tft.print("BATT MAX TEMP:");
    tft.setCursor(0,YVOLTAGE);
    tft.print("VOLTAGE:");
    tft.setCursor(0,YCSMTEMPS);
    tft.print("CSM:");
    tft.setCursor(0,YCHRGSTAT);
    tft.print("CHARGE STATUS:");
    if(readFrameFileExists==1){
      tft.setCursor(0,YSDREADSTAT);
      tft.print("SD READ:");
    }
  }
  tft.setCursor(XCANSTAT, YCANSTAT);
  tft.setTextColor(ILI9341_BLACK,ILI9341_RED);
  tft.setTextSize(1);
  if (battType==VALENCE_REV3){
    if (BMSrev3CANok()){
      tempCANstatus = "   VALENCE CAN OK";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
  }
  else if (battType == CUMMINS_REV1){
    if (CumminsCANok()){
      tempCANstatus = "   CUMMINS CAN OK";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
  }
  else if (battType == INVNTS_OLD){
    if (InvntsOldCANok()){
      tempCANstatus = "   INVNTUS CAN OK";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
  }
  else if (battType == INVNTS_60AH){
    if (Invnts60AhCANok()){
      tempCANstatus = "   INVNTUS CAN OK";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
  }
  else if (battType == INVNTS_80AH){
    if (Invnts80AhCANok()){
      tempCANstatus = "   INVNTUS CAN OK";
      tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
    }
  }
  tft.print(tempCANstatus);


  tft.setTextColor(ILI9341_WHITE,ILI9341_BLACK);
//-------------------------------------SOC--------------------------------------
  if (moduleSOCscale < 100.0){
    if (moduleSOCscale < 10.0){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  tempString = String::format("%.1f",moduleSOCscale);
  tft.setCursor(XSOC, YSOC); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat("%");
  tft.print(tempSpaces);
//------------------------------------/SOC/-------------------------------------

//-----------------------------------CURRENT------------------------------------
  tempSpaces = "";
  if (abs(battCurrent) < 100){
    if (abs(battCurrent) < 10){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (battCurrent >= 0){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%.1f",battCurrent);
  tft.setCursor(XCURRENT, YCURRENT); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat("A");
  tft.print(tempSpaces);
//-----------------------------------/CURRENT/----------------------------------

//--------------------------------MIN TEMPERATURE-------------------------------
  tempSpaces = "";
  if (abs(moduleMinTemperature) < 100){
    if (abs(moduleMinTemperature) < 10){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (moduleMinTemperature >= 0){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%.0f",moduleMinTemperature);
  tft.setCursor(XMINTEMP, YMINTEMP); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat(" C");
  tft.print(tempSpaces);
//------------------------------/MIN TEMPERATURE/-------------------------------

//-------------------------------MAX TEMPERATURE--------------------------------
  tempSpaces = "";
  if (abs(moduleMaxTemperature) < 100){
    if (abs(moduleMaxTemperature) < 10){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (moduleMaxTemperature >= 0){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%.0f",moduleMaxTemperature);
  tft.setCursor(XMAXTEMP, YMAXTEMP); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat(" C");
  tft.print(tempSpaces);
//-----------------------------/MAX TEMPERATURE/--------------------------------

//----------------------------------VOLTAGE-------------------------------------
  tempSpaces = "";
  if (battVoltage < 100.0){
    if (battVoltage < 10.0){
      tempSpaces = " ";
    }
    else{
    tempSpaces = "";
    }
  }
  tempString = String::format("%.2f",battVoltage);
  tft.setCursor(XVOLTAGE, YVOLTAGE); tft.setTextSize(3);
  tempSpaces.concat(tempString); tempSpaces.concat("V");
  tft.print(tempSpaces);

//--------------------------------/VOTLAGE/-------------------------------------

//-----------------------------CSM TEMPERATURES---------------------------------
  tempSpaces = "";
  if (abs(CsmTemps[0]) < 1000){
    if (abs(CsmTemps[0]) < 100){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (CsmTemps[0] > -10){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%d",CsmTemps[0]/10);
  tft.setCursor(XCSMTEMPS, YCSMTEMPS); tft.setTextSize(3);
  tempSpaces.concat(tempString);
  tempLine.concat(tempSpaces);

  tempSpaces = "";
  if (abs(CsmTemps[1]) < 1000){
    if (abs(CsmTemps[1]) < 100){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (CsmTemps[1] > -10){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%d",CsmTemps[1]/10);
  tempSpaces.concat(tempString);
  tempLine.concat(tempSpaces);

  tempSpaces = "";
  if (abs(CsmTemps[2]) < 1000){
    if (abs(CsmTemps[2]) < 100){
      tempSpaces = "  ";
    }
    else{
    tempSpaces = " ";
    }
  }
  if (CsmTemps[2] > -10){
    tempSpaces.concat(" ");
  }
  tempString = String::format("%d",CsmTemps[2]/10);
  tempSpaces.concat(tempString);
  tempLine.concat(tempSpaces);

  tempLine.concat("C");
  tft.print(tempLine);
//-------------------------------/CSM TEMPERATURES/-----------------------------

//---------------------------------CHARGE STATUS--------------------------------
  tempSpaces = "";
  tft.setCursor(XCHRGSTAT, YCHRGSTAT); tft.setTextSize(2);
  if (chargeStatus == 0){
    tempSpaces.concat("        MAIN");
  }
  else if (chargeStatus == 1){
    tempSpaces.concat("EQUALIZATION");
  }
  else if (chargeStatus == 2){
    tempSpaces.concat("       FLOAT");
  }
  else if (chargeStatus == 3){
    tempSpaces.concat("          NA");
  }
  else if (chargeStatus == 4){
    tempSpaces.concat("    COMPLETE");
  }
  tft.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);
  tft.print(tempSpaces);
//--------------------------------/CHARGE STATUS/-------------------------------

//------------------------------SD READ STATUS----------------------------------
  if(readFrameFileExists==1){
    tempSpaces = "";
    if (abs(CurrLoadFrame.Power) < 10000){
      if (abs(CurrLoadFrame.Power) < 1000){
        if (abs(CurrLoadFrame.Power) < 100){
          if (abs(CurrLoadFrame.Power) < 10){
            if (CurrLoadFrame.Power == 0){
              tempSpaces = "     ";
            }
            else{
            tempSpaces = "    ";
            }
          }
          else{
          tempSpaces = "   ";
          }
        }
        else{
        tempSpaces = "  ";
        }
      }
      else{
      tempSpaces = " ";
      }
    }
    if (CurrLoadFrame.Power > 0){
      tempSpaces.concat(" ");
    }

    tft.setCursor(XSDREADSTAT, YSDREADSTAT); tft.setTextSize(2);
    tempString = String::format("%d",CurrLoadFrame.Power);
    tempSpaces.concat(tempString);
    tempSpaces.concat(" W");
    tft.setTextColor(ILI9341_YELLOW,ILI9341_BLACK);
    tft.print(tempSpaces);
  }
//-----------------------------/SD READ STATUS/---------------------------------

//-------------------------------WIFI STATUS------------------------------------
  tempSpaces = " NO WIFI ";
  tft.setTextColor(ILI9341_BLACK,ILI9341_RED);
  tft.setCursor(XWIFISTAT, YWIFISTAT); tft.setTextSize(1);
  if (SSIDname != ""){
    tempSpaces = SSIDname;
    tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
  }
  tft.print(tempSpaces);
//------------------------------/WIFI STATUS/-----------------------------------

//----------------------------SPARK CLOUD STATUS--------------------------------
  tempSpaces = "NO INTERNET    ";
  tft.setTextColor(ILI9341_BLACK,ILI9341_RED);
  tft.setCursor(XNETSTAT, YNETSTAT); tft.setTextSize(1);
  if (SparkNetOk == 1){
    tempSpaces = "CLOUD CONNECTED";
    tft.setTextColor(ILI9341_BLACK,ILI9341_GREEN);
  }
  tft.print(tempSpaces);
//--------------------------/SPARK CLOUD STATUS/--------------------------------

  counts++;
}

void dispTrac(){

}
