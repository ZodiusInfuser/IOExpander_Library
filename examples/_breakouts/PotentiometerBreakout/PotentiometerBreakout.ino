/*
 * Potentiometer Breakout example (updated October 3, 2020)
 * 
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/potentiometer.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

/***** Library Includes *****/
#include <PotBreakout.h>
#include "HSVtoRGB.h"

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 30;

/***** Global Variables *****/
PotBreakout pot(Wire, 0x0E);


////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  delay(1000);

  Wire.begin();
  if(!pot.initialise())
  {
    while(true)
      delay(10);
  }

  //Uncomment this if running the breakout off a 5V supply
  //pot.setAdcVref(5.0f); 

  //Uncomment this to reverse the direction of the potentiometer to counter-clockwise
  //pot.setDirection(false);

  //Uncomment this to change the maximum brightness of the LED (from 0 to 1.0)
  //pot.setBrightness(0.5f);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  float analog = pot.readAsPercent();

  float h = (analog * 240.0f);
  byte r, g, b;
  HSVtoRGB(h, 100.0f, 100.0f, r, g, b);

  pot.setRGB(r, g, b);

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
