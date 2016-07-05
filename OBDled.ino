/*************************************************************************
* OBDled
* 
* Based on 
* (C) 2013 Stanley Huang <stanleyhuangyc@gmail.com> [OBD]
* (C) 2015 Rinky-Dink Electronics, Henning Karlsen, http://www.RinkyDinkElectronics.com [Screen]
*************************************************************************/

/* OBD UART */
#include <OBD.h>
#include "datalogger.h"
#define STATE_SD_READY  0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING  0x20
/* 
 * posible pid
 * static byte pidTier1[] = {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
 * static byte pidTier2[] = {PID_INTAKE_MAP, PID_MAF_FLOW, PID_TIMING_ADVANCE};
 * static byte pidTier3[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL};
 */
static byte pidTier1[] = {PID_RPM, PID_SPEED};
static byte pidTier2[] = {PID_THROTTLE};
static byte pidTier3[] = {PID_INTAKE_TEMP};

/* LCD TFT touch-enabled screen */
#include <UTFT.h>
#include <ITDB02_Touch.h>
#include <UTFT_Buttons_ITDB.h>
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];
// SainSmart screen on Arduino Mega
UTFT myGLCD(SSD1289,38,39,40,41);
ITDB02_Touch myTouch(6,5,4,3,2);
UTFT_Buttons myButtons(&myGLCD, &myTouch);
// buttons
int buttonMode, buttonLeds, buttonPerf, buttonOthr, buttonPressed;
int bMode = 0;
int bLeds = 6;
char* strMode[] = {"rpm3", "rpm2", "rpm1", "on 2", "on 1"};
char* strLeds[] = {"red ", "yllw", "gren", "cyan", "blue", "prpl", "whte", "off "};

/* Neopixel compatible RGB LED strip */
#include <Adafruit_NeoPixel.h>
// pin
#define STRIP_PIN1    8
#define STRIP_PIN2    9
// ammount of leds
#define MIN_PIXELS    0
#define RPM_PIXELS    30
#define SHIFT_PIXELS  5
#define TOTAL_PIXELS  30
// revolutions per minute
#define MIN_RPM       0
#define SHORT_RPM     3000
#define SHIFT_RPM     3500
#define MAX_RPM       4000

// original logger program
static uint8_t lastPid = 0;
static int lastValue = 0;

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

// logged values
int rpmVal            = 0; 
int speedVal          = 0; 
int throttleVal       = 0;
int tempVal           = 0;
int prevRpmValLcd     = 0;
int prevSpeedValLcd   = 0;

// performance
#define TARGET_DISTANCE 200
boolean measuringPerformance = true;
int prevSpeedVal    = 0;
float distance      = 0;
long speedTime      = 0;
long firstSpeedTime = 0;
long prevSpeedTime  = 0;
long currPerfTime   = 0;
long bestPerfTime   = 59999;

// leds
Adafruit_NeoPixel ledstrip1 = Adafruit_NeoPixel(TOTAL_PIXELS, STRIP_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledstrip2 = Adafruit_NeoPixel(TOTAL_PIXELS, STRIP_PIN2, NEO_GRB + NEO_KHZ800);

uint32_t OFF_COLOR      = ledstrip1.Color(  0,  0,  0);
uint32_t SHORT_COLOR    = ledstrip1.Color(  0,120,  0);
uint32_t SHIFT_COLOR    = ledstrip1.Color(  0,  0,120);

uint32_t ALL_COLOR[] =  { ledstrip1.Color(120,  0,  0),  //red
                          ledstrip1.Color( 60, 60,  0),  //yellow
                          ledstrip1.Color(  0,120,  0),  //green
                          ledstrip1.Color(  0, 60, 60),  //cyan
                          ledstrip1.Color(  0,  0,120),  //blue
                          ledstrip1.Color( 60,  0, 60),  //magenta
                          ledstrip1.Color( 40, 40, 40),  //white
                          ledstrip1.Color(  0,  0,  0)   //off
                        };
uint32_t MAT_COLOR[3][8] = {
  { ledstrip1.Color(240,  0,  0),  //red
    ledstrip1.Color(180,180,  0),  //yellow
    ledstrip1.Color(  0,240,  0),  //green
    ledstrip1.Color(  0,180,180),  //cyan
    ledstrip1.Color(  0,  0,240),  //blue
    ledstrip1.Color(180,  0,180),  //magenta
    ledstrip1.Color(120,120,120),  //white
    ledstrip1.Color(  0,  0,  0)   //off
  },
  { ledstrip1.Color(180,  0,  0),  //red
    ledstrip1.Color(135,135,  0),  //yellow
    ledstrip1.Color(  0,180,  0),  //green
    ledstrip1.Color(  0,135,135),  //cyan
    ledstrip1.Color(  0,  0,180),  //blue
    ledstrip1.Color(135,  0,135),  //magenta
    ledstrip1.Color( 90, 90, 90),  //white
    ledstrip1.Color(  0,  0,  0)   //off
  },
  { ledstrip1.Color(120,  0,  0),  //red
    ledstrip1.Color( 90, 90,  0),  //yellow
    ledstrip1.Color(  0,120,  0),  //green
    ledstrip1.Color(  0, 90, 90),  //cyan
    ledstrip1.Color(  0,  0,120),  //blue
    ledstrip1.Color( 90,  0, 90),  //magenta
    ledstrip1.Color( 60, 60, 60),  //white
    ledstrip1.Color(  0,  0,  0)   //off
  }
};

// strip
static byte rpmLedList[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};
static byte shiftLedList[] = {0,1,2,3,4};

boolean updateLeds = false;
int rpmLed   = 0;

//byte pidValue[TIER_NUM1];

long timeNext = 1000;
int updatesPerSecond = 0;

class COBDLogger : public COBD, public CDataLogger
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        myGLCD.clrScr();
        
        do {
            showStates();
        } while (!init());

        state |= STATE_OBD_READY;

        showStates();

        initScreen();
    }
    void loop()
    {

        // TO DO: Consider changing to
        // byte read(const byte pid[], byte count, int result[]);
        // to read multiple PID at the same time
        /* 
          Las variables static funcionan como en C, una vez definidas su valor se conserva por el resto del programa
          aunque la ejecucion de la funcion en la que se encuentran termine, y esta vuelva a ser llamada.

          Se utilizan de esta forma para aprovechar que el loop es un ciclo, y no colocar los condicionales de abajo
          dentro de uno.
        */
        static byte index = 0;
        //static byte index2 = 0;
        //static byte index3 = 0;
        
        /*
          Lee un tier 1 cada vez
          Si termino de leer los t1 lee un tier 2 una vez
          Si termino de leer los t2 lee un tier 3 una vez
        */
        logOBDData(pidTier1[index++]);
        /*
        if (index == TIER_NUM1) {
            index = 0;
            if (index2 == TIER_NUM2) {
                index2 = 0;
                logOBDData(pidTier3[index3]);
                index3 = (index3 + 1) % TIER_NUM3;
            } else {
                logOBDData(pidTier2[index2++]);
            }
        }
        */
        if (index == TIER_NUM1) index = 0;

        if (errors >= 2) {
            reconnect();
        }
    }
    void initScreen()
    {
        initLoggerScreen();
    }
private:
    byte state;
    void dataIdleLoop()
    {
        if (lastPid) {
            showLoggerData(lastPid, lastValue);
            lastPid = 0;
        }
    }
    int logOBDData(byte pid)
    {
        int value = 0;
        // send a query to OBD adapter for specified OBD-II pid
        if (read(pid, value)) {
            dataTime = millis();
            lastValue = value;
            lastPid = pid;
        }
        return value;
    }   
    void reconnect()
    {
        //startTime = millis();
        state &= ~(STATE_OBD_READY | STATE_ACC_READY);
        state |= STATE_SLEEPING;
        for (uint16_t i = 0; ; i++) {
            if (init()) {
                int value;
                if (read(PID_RPM, value) && value > 0) break;
            }
        }
        state &= ~STATE_SLEEPING;
        recover();
        setup();
    }
    
    // Show if OBD sensor and data acquisition are ready
    void showStates()
    {
        //myGLCD.clrScr();
      
        myGLCD.setColor(VGA_WHITE);
        myGLCD.setBackColor(VGA_BLACK);
        myGLCD.setFont(BigFont);

        myGLCD.print("OBDled by cuprumbot", 8, 20);
        myGLCD.print("Initializing...", 8, 60);

        myGLCD.print("OBD", 20, 160);
        myGLCD.print("ACC", 20, 200);

        if (state & STATE_OBD_READY) myGLCD.print("OK", 100, 160);
        if (state & STATE_ACC_READY) myGLCD.print("OK", 100, 200);

        if (myTouch.dataAvailable()) {
          buttonPressed = myButtons.checkButtons();
          bLeds = ++bLeds % 8;
          updateLeds = true;
        }

        uint32_t color = MAT_COLOR[2][bLeds];
        // TO DO: This is a good place to test the strips
        for (int i=0; i<30; i++) ledstrip1.setPixelColor(rpmLedList[i], color);
        for (int i=0; i<30; i++) ledstrip2.setPixelColor(rpmLedList[i], color);
        ledstrip1.show();
        ledstrip2.show();
    }
    
    void showLoggerData(byte pid, int value)
    {   
        updatesPerSecond++;
        
        switch (pid) {
        case PID_RPM:

            rpmVal = (unsigned int)value % 10000;
            rpmLed = map(rpmVal, MIN_RPM, MAX_RPM, MIN_PIXELS, RPM_PIXELS);

            if (abs(rpmVal - prevRpmValLcd) > 50) {              
              myGLCD.setFont(SevenSegNumFont);
  
              if (rpmVal < 1000) {
                myGLCD.setColor(VGA_BLACK);
                myGLCD.fillRect(10,30,41,79);
                myGLCD.setColor(VGA_WHITE);
                myGLCD.printNumI(rpmVal, 42, 30, 3);
              } else {
                if (rpmVal < 3000) myGLCD.setColor(VGA_WHITE);
                else if (rpmVal < 3500) myGLCD.setColor(VGA_YELLOW);
                else myGLCD.setColor(VGA_RED);
                myGLCD.printNumI(rpmVal, 10, 30, 4);
              }

              prevRpmValLcd = rpmVal;
              if (bMode < 3) updateLeds = true;
            }
            
            break;
            
        case PID_SPEED:

            prevSpeedVal = speedVal;
            speedVal = (unsigned int)value % 1000; 

            if (prevSpeedValLcd != speedVal) {
              myGLCD.setFont(SevenSegNumFont);
              
              if (speedVal < 10) {
                myGLCD.setColor(VGA_BLACK);
                myGLCD.fillRect(42,110,105,159);
                myGLCD.setColor(VGA_WHITE);
                myGLCD.printNumI(speedVal, 106, 110, 1);
              } else if (speedVal < 100) {
                myGLCD.setColor(VGA_BLACK);
                myGLCD.fillRect(42,110, 73,159);
                myGLCD.setColor(VGA_WHITE);
                myGLCD.printNumI(speedVal,  74, 110, 2);
              } else {
                myGLCD.printNumI(speedVal,  42, 110, 3);
              }

              prevSpeedValLcd = speedVal;
            }

            if (speedVal == 0) {
              measuringPerformance = true;
              distance = 0;
              speedTime = millis();
              firstSpeedTime = speedTime;          
            } else if (measuringPerformance) {
              prevSpeedTime = speedTime;
              speedTime = millis();
              // speedVal + prevSpeedVal / 2  --> average speed in km/h
              // speedTime - prevSpeedTime    --> elapsed time in ms
              distance += (speedVal + prevSpeedVal) * (speedTime - prevSpeedTime) / 7200;

              myGLCD.setFont(BigFont);
              myGLCD.printNumF(distance,1,230,130,'.',5);
              
              if (distance > TARGET_DISTANCE) {
                measuringPerformance = false;

                currPerfTime = speedTime - firstSpeedTime;
                if (currPerfTime < bestPerfTime) bestPerfTime = currPerfTime;

                myGLCD.setFont(BigFont);
                //myGLCD.printNumI(bestPerfTime, 220, 160, 5);
                myGLCD.printNumF((float)(bestPerfTime / 1000),2,230,90,'.',5);
                if (currPerfTime < 59999) {
                  //myGLCD.printNumI(currPerfTime, 220, 100, 5);
                  myGLCD.printNumF((float)(currPerfTime / 1000),2,230,50,'.',5);
                } else {
                  myGLCD.print(" OVER", 230, 50);
                }
              }
            }
            
            break;
            
        /*
        case PID_THROTTLE:
            throttleVal = value % 100;
            break;
        case PID_INTAKE_TEMP:
            tempVal = value;
            break;
        */
        }

        if (updateLeds) {
          updateLeds = false;

          if (bMode < 3) {
            uint32_t color = MAT_COLOR[bMode][bLeds];

            // Illuminates LEDs according to RPM
            for (int i=0; i<rpmLed && i<RPM_PIXELS; i++) {
              ledstrip1.setPixelColor(rpmLedList[i], color);
              ledstrip2.setPixelColor(rpmLedList[i], color);
            }
            for (int i=rpmLed; i<RPM_PIXELS; i++) {
              ledstrip1.setPixelColor(rpmLedList[i], OFF_COLOR);
              ledstrip2.setPixelColor(rpmLedList[i], OFF_COLOR);
            }
/*        
          } else if (bMode == 1) {

            // Illuminates LEDs according to RPM and signals a good shift
            for (int i=0; i<rpmLed && i<RPM_PIXELS; i++) {
              ledstrip1.setPixelColor(rpmLedList[i], ALL_COLOR[bLeds]);
              ledstrip2.setPixelColor(rpmLedList[i], ALL_COLOR[bLeds]);
            }
            for (int i=rpmLed; i<RPM_PIXELS; i++) {
              ledstrip1.setPixelColor(rpmLedList[i], OFF_COLOR);
              ledstrip2.setPixelColor(rpmLedList[i], OFF_COLOR);
            }
            if        (rpmVal > SHIFT_RPM) {
              for (int i=0; i<SHIFT_PIXELS; i++) {
                ledstrip1.setPixelColor(shiftLedList[i], SHIFT_COLOR);
                ledstrip2.setPixelColor(shiftLedList[i], SHIFT_COLOR);
              }
            } else if (rpmVal > SHORT_RPM) {
              for (int i=0; i<SHIFT_PIXELS; i++) {
                ledstrip1.setPixelColor(shiftLedList[i], SHORT_COLOR);
                ledstrip2.setPixelColor(shiftLedList[i], SHORT_COLOR);
              }
            }
*/ 
          } else {
            uint32_t color = MAT_COLOR[bMode-2][bLeds];

            // Turns on all LEDs
            for (int i=0; i<TOTAL_PIXELS; i++) {
              ledstrip1.setPixelColor(rpmLedList[i], color);
              ledstrip2.setPixelColor(rpmLedList[i], color);
            }   
          }
          
          ledstrip1.show();
          ledstrip2.show();
        }

        if (millis() > timeNext) {
          myGLCD.setFont(BigFont);
          myGLCD.setColor(VGA_WHITE);
          myGLCD.printNumI(updatesPerSecond, 278, 10, 2);
          
          updatesPerSecond = 0;
          timeNext = millis() + 1000;
        }
    }

    void initLoggerScreen()
    {
        myGLCD.clrScr();
      
        myGLCD.setColor(VGA_WHITE);
        myGLCD.setBackColor(VGA_BLACK);
        myGLCD.setFont(BigFont);
        
        myGLCD.print("RPM", 10, 10);
        myGLCD.print("KMH", 10, 90);
        
        myGLCD.print("FPS", 170,  10);
        //myGLCD.print(" ", 170,  30);
        myGLCD.print("CUR", 170,  50);
        //myGLCD.print(" ", 170,  70);
        myGLCD.print("BST", 170,  90);
        //myGLCD.print(" ", 170, 110);
        myGLCD.print("DIS", 170, 130);
        
        myButtons.setButtonColors(VGA_WHITE, VGA_GRAY, VGA_WHITE, VGA_RED, VGA_NAVY);
        myButtons.drawButtons();
    }
};

static COBDLogger logger;

void setup() {
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(BigFont);

  myTouch.InitTouch(1);
  myTouch.setPrecision(PREC_LOW);

  myButtons.setTextFont(BigFont);
  buttonMode = myButtons.addButton(  4, 200, 75, 30, "MODE");
  buttonLeds = myButtons.addButton( 83, 200, 75, 30, "LEDS");
  buttonPerf = myButtons.addButton(162, 200, 75, 30, "PERF");
  buttonOthr = myButtons.addButton(241, 200, 75, 30, "OTHR", BUTTON_DISABLED);
  
  ledstrip1.begin();
  ledstrip2.begin();
  
  logger.begin();
  logger.initSender();

  logger.setup();
}

void loop() {
  if (myTouch.dataAvailable()) {
    buttonPressed = myButtons.checkButtons();
    myGLCD.setFont(BigFont);

    if        (buttonPressed == buttonLeds) {
      bLeds = ++bLeds % 8;
      myGLCD.print(strLeds[bLeds], 89,170);
      updateLeds = true;
    } else if (buttonPressed == buttonMode) {
      bMode = ++bMode % 5;
      myGLCD.print(strMode[bMode], 10,170);
      updateLeds = true;
    } else if (buttonPressed == buttonPerf) {
      bestPerfTime = 59999;
      myGLCD.printNumF((float)(bestPerfTime / 1000),2,230,90,'.',5);
    }
  }
  
  logger.loop();
}
