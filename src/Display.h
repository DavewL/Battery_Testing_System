#define FOUR_LINE_LCD 0
#define THREEPOINTTWO_INCH_NEXTION 1
#define TWOFORTY_BY_THREETWENTY_SERIAL 2

#define TFT_DC D7
#define TFT_CS D0
#define TFT_RST D6

#define YBATTHEADING 0 //5
#define XSOC 206
#define YSOC 30 //40
#define XCURRENT 190
#define YCURRENT 60 //72
#define XMINTEMP 206
#define YMINTEMP 90 //102
#define XMAXTEMP 206
#define YMAXTEMP 120 //132
#define XVOLTAGE 206
#define YVOLTAGE 150 //162
#define XCSMTEMPS 80
#define YCSMTEMPS 180 //192
#define XCHRGSTAT 155
#define YCHRGSTAT 205
#define XCANSTAT 216
#define YCANSTAT 12
#define XSDREADSTAT 100
#define YSDREADSTAT 222
#define XWIFISTAT 206
#define YWIFISTAT 222
#define XNETSTAT 206
#define YNETSTAT 232

void initDisplay(int dispType);

void dispBattery(void);

void dispBatteryHeading(void);
void dispBatterySOC(void);
void dispBatteryVolt(void);
void dispBatteryAmps(void);
void dispBatteryStats(void);
void dispBatteryTemps(void);
void dispBatteryCSM(void);
void dispBatterySDPwr(void);

void dispTrac(void);
