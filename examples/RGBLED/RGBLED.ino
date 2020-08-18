/*
 * IOExpander rgb led example (updated August 18, 2020)
 * 
 * Demonstrates running a common-cathode RGB LED, or trio of LEDs wired between each PWM pin and Ground.
 * You must wire your Red, Green and Blue LEDs or LED elements to pins 1, 3 and 5.
 * 
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/rgbled.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

/***** Project Includes *****/
#include "RGBLED.h"

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 30;

static const byte LED_R = 1;
static const byte LED_G = 3;
static const byte LED_B = 5;

/***** Global Variables *****/
IOExpander ioe(Wire, 0x18);
RGBLED rgbled(ioe, LED_R, LED_G, LED_B, 0.5f, true);


////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  delay(1000);

  Wire.begin();
  if(!rgbled.initialise())
  {
    while(true)
      delay(10);
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  float h = fmodf(((float)millis() * 360.0f) / 10000.0f, 360.0f);
  rgbled.setHue(h, 100.0f, 100.0f);
  
  delay(DELAY_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
