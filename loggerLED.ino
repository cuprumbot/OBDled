/*************************************************************************
* Arduino GPS/OBD-II/G-Force Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#include <SPI.h>
#include "MicroLCD.h"
#include "images.h"
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"
#include <Adafruit_NeoPixel.h>

//Pin del Anillo Led
#define STRIP_PIN 5

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING 0x20

#define MIN_PIXELS 0
#define MAX_PIXELS 16
#define SHIFT_PIXELS 12
#define TOTAL_PIXELS 64

#define MIN_SPEED 0
#define MAX_SPEED 150
#define MIN_RPM 0
#define MAX_RPM 5000
#define SHORT_RPM 3000
#define SHIFT_RPM 3500
#define OVER_REV_RPM 4500

Adafruit_NeoPixel ledstrip = Adafruit_NeoPixel(TOTAL_PIXELS, STRIP_PIN, NEO_GRB + NEO_KHZ800);

static int lastSpeed = -1;
static int speed = 0;
static uint32_t distance = 0;
static uint32_t startTime = 0;
static uint8_t lastPid = 0;
static int lastValue = 0;

uint32_t TEST_COLOR     = ledstrip.Color(20, 20, 20);
uint32_t RPM_COLOR      = ledstrip.Color(20, 0, 0);
uint32_t OFF_COLOR      = ledstrip.Color(0, 0, 0);
uint32_t SHORT_COLOR    = ledstrip.Color(0, 20, 0);
uint32_t SHIFT_COLOR    = ledstrip.Color(0, 0, 40);
uint32_t OVER_REV_COLOR = ledstrip.Color(60, 60, 60);

int rpmVal      = 0, rpmLed   = MAX_PIXELS+1, rpmPrevLed   = -1;
int speedVal    = 0, speedLed = MAX_PIXELS+1, speedPrevLed = -1;
int throttleVal = 0;
int tempVal     = 0;

static byte rpmLedList[] = {4,3, 12,11, 20,19, 28,27, 36,35, 44,43, 52,51, 60,59};
static byte shiftLedList[] = {24,25,26, 29,30,31, 32,33,34, 37,38,39};

boolean rpmIncreasing = false;
boolean updateLeds = false;

/*
static byte pidTier1[] = {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static byte pidTier2[] = {PID_INTAKE_MAP, PID_MAF_FLOW, PID_TIMING_ADVANCE};
static byte pidTier3[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL};
*/
static byte pidTier1[] = {PID_RPM, PID_SPEED};
static byte pidTier2[] = {PID_THROTTLE};
static byte pidTier3[] = {PID_INTAKE_TEMP};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

byte pidValue[TIER_NUM1];

class COBDLogger : public COBD, public CDataLogger
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        showStates();

        do {
            showStates();
        } while (!init());

        state |= STATE_OBD_READY;

        showStates();

        initScreen();
    }
    void loop()
    {
        /* 
          Las variables static funcionan como en C, una vez definidas su valor se conserva por el resto del programa
          aunque la ejecucion de la funcion en la que se encuentran termine, y esta vuelva a ser llamada.

          Se utilizan de esta forma para aprovechar que el loop es un ciclo, y no colocar los condicionales de abajo
          dentro de uno.
        */
        static byte index = 0;
        static byte index2 = 0;
        static byte index3 = 0;
        
        /*
          Lee un tier 1 cada vez
          Si termino de leer los t1 lee un tier 2 una vez
          Si termino de leer los t2 lee un tier 3 una vez
        */
        logOBDData(pidTier1[index++]);
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
            // log data to SD card
            logData(0x100 | pid, value);
            lastValue = value;
            lastPid = pid;
        }
        return value;
    }   
    void reconnect()
    {
        lcd.clear();
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print("Reconnecting");
        startTime = millis();
        state &= ~(STATE_OBD_READY | STATE_ACC_READY);
        state |= STATE_SLEEPING;
        for (uint16_t i = 0; ; i++) {
            if (i == 5) {
                lcd.backlight(false);
                lcd.clear();
            }
            if (init()) {
                int value;
                if (read(PID_RPM, value) && value > 0)
                    break;
            }
        }
        state &= ~STATE_SLEEPING;
        recover();
        setup();
    }
    void showTickCross(bool yes)
    {
        lcd.draw(yes ? tick : cross, 16, 16);
    }
    
    // Show if OBD sensor and data acquisition are ready
    void showStates()
    {
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0, 4);
        lcd.print("OBD ");
        showTickCross(state & STATE_OBD_READY);
        lcd.setCursor(0, 6);
        lcd.print("ACC ");
        showTickCross(state & STATE_ACC_READY);

        // Test the strips
        for (int i=0;i<TOTAL_PIXELS;i++){
          ledstrip.setPixelColor(i, OFF_COLOR);
        }
        for(int i=0;i<MAX_PIXELS;i++){
          //ledstrip.setPixelColor(i, TEST_COLOR);
          ledstrip.setPixelColor(rpmLedList[i], TEST_COLOR);         
        }
        ledstrip.show();
        
    }
    
    void showLoggerData(byte pid, int value)
    {
        char buf[8];
        switch (pid) {
        case PID_RPM:
            lcd.setCursor(64, 0);
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.printInt((unsigned int)value % 10000, 4);

            rpmVal = (unsigned int)value % 10000;
            rpmPrevLed = rpmLed;
            rpmLed = map(rpmVal, MIN_RPM, MAX_RPM, MIN_PIXELS, MAX_PIXELS);
            rpmIncreasing = (rpmLed > rpmPrevLed);
            updateLeds = true;
            
            break;
        case PID_SPEED:
            if (lastSpeed != value) {
                lcd.setCursor(0, 0);
                lcd.setFontSize(FONT_SIZE_XLARGE);
                lcd.printInt((unsigned int)value % 1000, 3);
                lastSpeed = value;
            }

            speedVal = (unsigned int)value % 1000; 
            break;
        // MINIMAL TEST
        /*
        case PID_THROTTLE:
            lcd.setCursor(24, 5);
            lcd.setFontSize(FONT_SIZE_SMALL);
            lcd.printInt(value % 100, 3);

            throttleVal = value % 100;
            break;
        case PID_INTAKE_TEMP:
            if (value < 1000) {
                lcd.setCursor(102, 5);
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.printInt(value, 3);
            }

            tempVal = value;
            break;
        */
        }

        if (updateLeds) {
          for (int i=0; i<rpmLed && i<MAX_PIXELS; i++) ledstrip.setPixelColor(rpmLedList[i], RPM_COLOR);
          for (int i=rpmLed; i<MAX_PIXELS; i++) ledstrip.setPixelColor(rpmLedList[i], OFF_COLOR);

          if      (rpmVal > SHORT_RPM)    for (int i=0; i<SHIFT_PIXELS; i++) ledstrip.setPixelColor(shiftLedList[i], SHORT_COLOR);
          else if (rpmVal > SHIFT_RPM)    for (int i=0; i<SHIFT_PIXELS; i++) ledstrip.setPixelColor(shiftLedList[i], SHIFT_COLOR);
          else if (rpmVal > OVER_REV_RPM) for (int i=0; i<SHIFT_PIXELS; i++) ledstrip.setPixelColor(shiftLedList[i], OVER_REV_COLOR);
          else                            for (int i=0; i<SHIFT_PIXELS; i++) ledstrip.setPixelColor(shiftLedList[i], OFF_COLOR);
          
          ledstrip.show();
        }
    }

    void initLoggerScreen()
    {
        lcd.clear();
        lcd.backlight(true);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.setCursor(24, 3);
        lcd.print("km/h");
        lcd.setCursor(110, 3);
        lcd.print("rpm");
        // MINIMAL TEST
        /*
        lcd.setCursor(0, 5);
        lcd.print("THR:   %");
        lcd.setCursor(80, 5);
        lcd.print("AIR:   C");
        */
    }
};

static COBDLogger logger;

void setup()
{
    ledstrip.begin();
  
    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.println("loggerLED");

    logger.begin();
    logger.initSender();

    logger.setup();
}

void loop()
{
    logger.loop();
}
