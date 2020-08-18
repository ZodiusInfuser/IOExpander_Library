/*
 * IOExpander pwm example (updated August 18, 2020)
 * 
 * Demonstrates running a common-cathode RGB LED, or trio of LEDs wired between each PWM pin and Ground.
 * You must wire your Red, Green and Blue LEDs or LED elements to pins 1, 3 and 5.
 * 
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/pwm.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

/***** Library Includes *****/
#include <IOExpander.h>
#include "HSVtoRGB.h"

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 30;

static const byte PIN_RED = 1;
static const byte PIN_GREEN = 3;
static const byte PIN_BLUE = 5;

static constexpr float BRIGHTNESS = 0.05f; //Effectively the maximum fraction of the period that the LED will be on
static constexpr unsigned int PERIOD = (unsigned int)(255.0f / BRIGHTNESS);  //Add a period large enough to get 0-255 steps at the desired brightness

static const bool INVERT_OUTPUT = true;  //true for common cathode, false for common anode

/***** Global Variables *****/
IOExpander ioe(Wire, 0x18);


////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  delay(1000);

  Wire.begin();
  if(!ioe.initialise())
  {
    while(true)
      delay(10);
  }

  ioe.setPwmPeriod(PERIOD);
  ioe.setPwmControl(1);  //PWM as fast as we can to avoid LED flicker

  ioe.setMode(PIN_RED, IOExpander::PIN_PWM, false, INVERT_OUTPUT);
  ioe.setMode(PIN_GREEN, IOExpander::PIN_PWM, false, INVERT_OUTPUT);
  ioe.setMode(PIN_BLUE, IOExpander::PIN_PWM, false, INVERT_OUTPUT);

  Serial.print("Running LED with ");
  Serial.print((unsigned int)((float)PERIOD * BRIGHTNESS));
  Serial.println(" brightness steps.");
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  float h = fmodf(((float)millis() * 360.0f) / 10000.0f, 360.0f);
    
  byte r, g, b;
  HSVtoRGB(h, 100.0f, 100.0f, r, g, b);
  
  ioe.output(PIN_RED, r);
  ioe.output(PIN_GREEN, g);
  ioe.output(PIN_BLUE, b);

  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.println(b);

  delay(DELAY_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
