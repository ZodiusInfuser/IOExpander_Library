/*
 * IOExpander button example (updated August 18, 2020)
 * 
 * Demonstrates handling a single button input using IO Expander.
 * Your button should be wired between pin 14 and ground on the IO Expander.
 * A pull-up resistor will be enabled, causing your button to read 1 normally, and 0 when pressed.
 *
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/button.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

/***** Library Includes *****/
#include <IOExpander.h>

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 30;

/***** Global Variables *****/
IOExpander ioe(Wire, 0x18);
bool lastValue = HIGH;


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
  
  ioe.setMode(14, IOExpander::PIN_IN_PU);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  bool value = ioe.input(14);
  if(value != lastValue)
  {
    if(value)
      Serial.println("Button has been released");
    else
      Serial.println("Button has been pressed");
    lastValue = value;
  }

  delay(DELAY_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
