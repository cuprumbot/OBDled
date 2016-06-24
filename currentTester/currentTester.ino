#include <UTFT.h>
#include <ITDB02_Touch.h>
#include <UTFT_Buttons_ITDB.h>

#include <Adafruit_NeoPixel.h>
//Pin del Anillo Led
#define RINGPIN        8
//Numero de Led Pixels
#define NUMPIXELS      64

Adafruit_NeoPixel ringled = Adafruit_NeoPixel(NUMPIXELS, RINGPIN, NEO_GRB + NEO_KHZ800);

// Declare which fonts we will be using
extern uint8_t SmallFont[];
extern uint8_t BigFont[];

// Set up UTFT...
UTFT          myGLCD(SSD1289,38,39,40,41);

// Set up UTouch...
ITDB02_Touch        myTouch(6,5,4,3,2);

// Finally we set up UTFT_Buttons :)
UTFT_Buttons  myButtons(&myGLCD, &myTouch);

int count = 64;
int light = 10;

boolean r = true;
boolean g = true;
boolean b = true;

void setup()
{
  myGLCD.InitLCD();
  myGLCD.clrScr();
  myGLCD.setFont(BigFont);

  myTouch.InitTouch(1);
  myTouch.setPrecision(PREC_MEDIUM);
  
  myButtons.setTextFont(BigFont);

  myGLCD.setColor(VGA_WHITE);
  myGLCD.setBackColor(VGA_BLACK);

  ringled.begin();
}

void loop()
{
  int but1, but2, but3, but4, butR, butG, butB, pressed_button;
  
  but1 = myButtons.addButton(  10,  10, 100, 70, "C+");
  but2 = myButtons.addButton( 110,  10, 100, 70, "C-");
  but3 = myButtons.addButton(  10,  80, 100, 70, "L+");
  but4 = myButtons.addButton( 110,  80, 100, 70, "L-");
  butR = myButtons.addButton(  10, 180,  66, 50, "R");
  butG = myButtons.addButton(  77, 180,  66, 50, "G");
  butB = myButtons.addButton( 143, 180,  66, 50, "B");
  myButtons.drawButtons();

  myGLCD.printNumI(count, 250,  10, 3);
  myGLCD.printNumI(light, 250,  60, 3);

  for (int i = 0; i < NUMPIXELS; i++) ringled.setPixelColor(i, ringled.Color(0,0,0));
  for (int i = 0; i < count; i++) ringled.setPixelColor(i, ringled.Color(light,light,light));
  ringled.show();

  while(1) 
  {
    if (myTouch.dataAvailable() == true)
    {
      pressed_button = myButtons.checkButtons();

      if (pressed_button==but1)
        count++;
      if (pressed_button==but2)
        count--;
      if (pressed_button==but3)
        light += 5;
      if (pressed_button==but4)
        light -= 5;
      if (pressed_button==butR)
        r = !r;
      if (pressed_button==butG)
        g = !g;
      if (pressed_button==butB)
        b = !b;

      if (count > 64) count = 64;
      if (count < 0) count = 0;

      if (light > 200) light = 200;
      if (light < 5) light = 5;

      myGLCD.printNumI(count, 250,  10, 3);
      myGLCD.printNumI(light, 250,  60, 3);

      for (int i = 0; i < NUMPIXELS; i++) ringled.setPixelColor(i, ringled.Color(0,0,0));
      uint32_t color = ringled.Color((r) ? light : 0, (g) ? light : 0, (b) ? light: 0);
      for (int i = 0; i < count; i++) ringled.setPixelColor(i, color);
      ringled.show();
    }
  }
}

