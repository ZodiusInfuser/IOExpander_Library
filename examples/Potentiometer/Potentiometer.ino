/*
 * IOExpander potentiometer example (updated August 18, 2020)
 * 
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/potentiometer.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

/***** Library Includes *****/
#include <IOExpander.h>
#include "HSVtoRGB.h"

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 30;

static const byte PIN_RED = 1;
static const byte PIN_GREEN = 7;
static const byte PIN_BLUE = 2;

static const byte POT_ENC_A = 12;
static const byte POT_ENC_B = 3;
static const byte POT_ENC_C = 11;

static constexpr float BRIGHTNESS = 0.5f; //Effectively the maximum fraction of the period that the LED will be on
static constexpr unsigned int PERIOD = (unsigned int)(255.0f / BRIGHTNESS);  //Add a period large enough to get 0-255 steps at the desired brightness

static const bool INVERT_OUTPUT = true; //true for common cathode, false for common anode

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

  ioe.setMode(POT_ENC_A, IOExpander::PIN_OUT);
  ioe.setMode(POT_ENC_B, IOExpander::PIN_OUT);
  ioe.setMode(POT_ENC_C, IOExpander::PIN_ADC);

  ioe.output(POT_ENC_A, 1);
  ioe.output(POT_ENC_B, 0);

  ioe.setPwmPeriod(PERIOD);
  ioe.setPwmControl(2);  //PWM as fast as we can to avoid LED flicker

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
  float analog = ioe.inputAsVoltage(POT_ENC_C);

  float h = (analog * 360.0f) / 3.3f;
  byte r, g, b;
  HSVtoRGB(h, 100.0f, 100.0f, r, g, b);
  
  ioe.output(PIN_RED, r);
  ioe.output(PIN_GREEN, g);
  ioe.output(PIN_BLUE, b);

  Serial.print(analog);
  Serial.print(", ");
  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.println(b);

  delay(DELAY_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
