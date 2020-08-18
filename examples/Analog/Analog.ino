/*
 * IOExpander analog example (updated August 18, 2020)
 * 
 * Demonstrates handling a single analog input using IO Expander.
 * You should wire a rotary potentiometer to pin 12.
 * The pot should be connected to 3.3v power and ground.
 *
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/analog.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

/***** Library Includes *****/
#include <IOExpander.h>

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 30;

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
  
  ioe.setAdcVref(3.3f);  //Input voltage of IO Expander, this is 3.3 on Breakout Garden
  ioe.setMode(12, IOExpander::PIN_ADC);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  float adc = ioe.inputAsVoltage(12);
  Serial.println(adc, 2);

  delay(DELAY_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
