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
int buttonMode, buttonLeds, buttonPerf, buttonOther, buttonPressed;
int bMode = 0;
int bLeds = 0;


/* Neopixel compatible RGB LED strip */
#include <Adafruit_NeoPixel.h>
// pin
#define STRIP_PIN1    3
#define STRIP_PIN2    4
// ammount of leds
#define MIN_PIXELS    0
#define RPM_PIXELS    29
#define SHIFT_PIXELS  1
#define TOTAL_PIXELS  30
// revolutions per minute
#define MIN_RPM       0
#define SHORT_RPM     2800
#define SHIFT_RPM     3200
#define OVER_REV_RPM  3600
#define MAX_RPM       4000

// original logger program
static int lastSpeed = -1;
static int speed = 0;
static uint8_t lastPid = 0;
static int lastValue = 0;

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

// logged values
int rpmVal      = 0; 
int speedVal    = 0; 
int throttleVal = 0;
int tempVal     = 0;

// performance
#define TARGET_DISTANCE 200
boolean measuringPerformance = true;
int prevSpeedVal    = 0;
float distance        = 0;
long speedTime      = 0;
long firstSpeedTime = 0;
long prevSpeedTime  = 0;
long currPerfTime   = 0;
long bestPerfTime   = 99999;

// leds
Adafruit_NeoPixel ledstrip1 = Adafruit_NeoPixel(TOTAL_PIXELS, STRIP_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledstrip2 = Adafruit_NeoPixel(TOTAL_PIXELS, STRIP_PIN2, NEO_GRB + NEO_KHZ800);

uint32_t TEST_COLOR     = ledstrip1.Color(20, 20, 20);
uint32_t RPM_COLOR      = ledstrip1.Color(20, 0, 0);
uint32_t OFF_COLOR      = ledstrip1.Color(0, 0, 0);
uint32_t SHORT_COLOR    = ledstrip1.Color(0, 20, 0);
uint32_t SHIFT_COLOR    = ledstrip1.Color(0, 0, 30);
uint32_t OVER_REV_COLOR = ledstrip1.Color(20, 20, 20);

uint32_t ALL_COLOR[] = {  ledstrip1.Color(20, 0, 0),
                          ledstrip1.Color( 0,20, 0),
                          ledstrip1.Color( 0, 0,20),
                          ledstrip1.Color(15,15,15)  };

// strip
static byte rpmLedList[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};
static byte shiftLedList[] = {0};

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
        
        // TO DO: This is a good place to test the strips
    }
    
    void showLoggerData(byte pid, int value)
    {      
        char buf[8];
        switch (pid) {
        case PID_RPM:

            rpmVal = (unsigned int)value % 10000;
            rpmLed = map(rpmVal, MIN_RPM, MAX_RPM, MIN_PIXELS, RPM_PIXELS);

            myGLCD.setFont(SevenSegNumFont);

            if (rpmVal < 1000) {
              myGLCD.setColor(VGA_BLACK);
              myGLCD.fillRect(20,40,51,89);
              myGLCD.setColor(VGA_WHITE);
              myGLCD.printNumI(rpmVal, 52, 40, 3);
            } else {
              myGLCD.printNumI(rpmVal, 20, 40, 4);
            }
            
            updateLeds = true;
            
            break;
        case PID_SPEED:

            prevSpeedVal = speedVal;
            speedVal = (unsigned int)value % 1000; 

            myGLCD.setFont(SevenSegNumFont);

            if (speedVal < 10) {
              myGLCD.setColor(VGA_BLACK);
              myGLCD.fillRect(52,120,115,169);
              myGLCD.setColor(VGA_WHITE);
              myGLCD.printNumI(speedVal, 116, 120, 1);
            } else if (speedVal < 100) {
              myGLCD.setColor(VGA_BLACK);
              myGLCD.fillRect(52,120,83,169);
              myGLCD.setColor(VGA_WHITE);
              myGLCD.printNumI(speedVal, 84, 120, 2);
            } else {
              myGLCD.printNumI(speedVal, 52, 120, 3);
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
              
              if (distance > TARGET_DISTANCE) {
                measuringPerformance = false;

                currPerfTime = speedTime - firstSpeedTime;
                if (currPerfTime < bestPerfTime) bestPerfTime = currPerfTime;

                myGLCD.setFont(BigFont);
                //myGLCD.print("PERF", 220, 80);
                //myGLCD.print("BEST", 220, 140);
                myGLCD.printNumI(currPerfTime, 220, 100, 5);
                myGLCD.printNumI(bestPerfTime, 220, 160, 5);
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
          
          for (int i=0; i<rpmLed && i<RPM_PIXELS; i++) ledstrip1.setPixelColor(rpmLedList[i], ALL_COLOR[bLeds]);
          for (int i=rpmLed; i<RPM_PIXELS; i++) ledstrip1.setPixelColor(rpmLedList[i], OFF_COLOR);

          if      (rpmVal > OVER_REV_RPM) for (int i=0; i<SHIFT_PIXELS; i++) ledstrip1.setPixelColor(shiftLedList[i], OVER_REV_COLOR);
          else if (rpmVal > SHIFT_RPM)    for (int i=0; i<SHIFT_PIXELS; i++) ledstrip1.setPixelColor(shiftLedList[i], SHIFT_COLOR);
          else if (rpmVal > SHORT_RPM)    for (int i=0; i<SHIFT_PIXELS; i++) ledstrip1.setPixelColor(shiftLedList[i], SHORT_COLOR);
          else                            for (int i=0; i<SHIFT_PIXELS; i++) ledstrip1.setPixelColor(shiftLedList[i], OFF_COLOR);

          for (int i=0; i<rpmLed && i<RPM_PIXELS; i++) ledstrip2.setPixelColor(rpmLedList[i], ALL_COLOR[bLeds]);
          for (int i=rpmLed; i<RPM_PIXELS; i++) ledstrip2.setPixelColor(rpmLedList[i], OFF_COLOR);

          if      (rpmVal > OVER_REV_RPM) for (int i=0; i<SHIFT_PIXELS; i++) ledstrip2.setPixelColor(shiftLedList[i], OVER_REV_COLOR);
          else if (rpmVal > SHIFT_RPM)    for (int i=0; i<SHIFT_PIXELS; i++) ledstrip2.setPixelColor(shiftLedList[i], SHIFT_COLOR);
          else if (rpmVal > SHORT_RPM)    for (int i=0; i<SHIFT_PIXELS; i++) ledstrip2.setPixelColor(shiftLedList[i], SHORT_COLOR);
          else                            for (int i=0; i<SHIFT_PIXELS; i++) ledstrip2.setPixelColor(shiftLedList[i], OFF_COLOR);

          updatesPerSecond++;
          ledstrip1.show();
          ledstrip2.show();
        }

        if (millis() > timeNext) {
          //myGLCD.setColor(VGA_WHITE);
          myGLCD.setFont(BigFont);
          //myGLCD.print("FPS", 220, 20);
          myGLCD.printNumI(updatesPerSecond, 268, 40, 2);
          
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
        
        myGLCD.print("RPM", 20, 20);
        myGLCD.print("KMH", 20, 100);
        
        myGLCD.print("FPS", 220, 20);
        
        myGLCD.print("PERF", 220, 80);
        myGLCD.print("BEST", 220, 140);
        myGLCD.print("    -", 220, 100);
        myGLCD.print("    -", 220, 160);
        
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

  myButtons.setTextFont(SmallFont);
  buttonMode = myButtons.addButton( 10, 200, 68, 30, "MODE");
  buttonLeds = myButtons.addButton( 88, 200, 67, 30, "LEDS");
  buttonPerf = myButtons.addButton(163, 200, 68, 30, "PERF");
  buttonOther = myButtons.addButton(243, 200, 67, 30, "OTHER");
  
  ledstrip1.begin();
  ledstrip2.begin();
  
  logger.begin();
  logger.initSender();

  logger.setup();
}

void loop() {
  if (myTouch.dataAvailable()) {
    buttonPressed = myButtons.checkButtons();

    if (buttonPressed == buttonLeds) {
      bLeds = ++bLeds % 4;
    }
  }
  
  logger.loop();  
}
