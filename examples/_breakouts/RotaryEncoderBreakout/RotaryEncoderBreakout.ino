/*
 * Rotary Encoder Breakout eample (updated October 3, 2020)
 * 
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/rotary.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

//Uncomment this to use the interrupt pin
//#define USE_INTERRUPT_PIN

//Uncomment this to use the interrupt routine
//#define USE_INTERRUPT_ROUTINE

/***** Library Includes *****/
#include <EncBreakout.h>
#include "HSVtoRGB.h"

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 30;

/***** Global Variables *****/
#if defined(USE_INTERRUPT_PIN) || defined(USE_INTERRUPT_ROUTINE)
  EncBreakout enc(Wire, 0x0F, EncBreakout::DEFAULT_BRIGHTNESS, 1, 4); //Interrupt pin
#else
  EncBreakout enc(Wire, 0x0F);  //I2C Polling
#endif
int16_t count = 0;

#if defined(USE_INTERRUPT_PIN) && defined(USE_INTERRUPT_ROUTINE)
  volatile bool intHandled = false;
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(USE_INTERRUPT_PIN) && defined(USE_INTERRUPT_ROUTINE)
  void interruptRoutine(void)
  {
    intHandled = true;
  }
#endif



////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  delay(1000);

  Wire.begin();
  if(!enc.initialise())
  {
    while(true)
      delay(10);
  }

  #if defined(USE_INTERRUPT_PIN) && defined(USE_INTERRUPT_ROUTINE)
    enc.setInterruptCallback(interruptRoutine);
  #endif

  //Uncomment this to reverse the direction of the encoder to counter-clockwise
  //enc.setDirection(false);

  //Uncomment this to change the maximum brightness of the LED (from 0 to 1.0)
  //enc.setBrightness(0.5f);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  #if defined(USE_INTERRUPT_PIN) && defined(USE_INTERRUPT_ROUTINE)
    if(intHandled)
    { 
      count = enc.read();
      intHandled = false;
    }
  #else
    if(enc.available())
    { 
      count = enc.read();
    }
  #endif
  
  while(count < 0)
    count += 24;
  
  float h = ((float)(count % 24) * 360.0f) / 24.0f;
  byte r, g, b;
  HSVtoRGB(h, 100.0f, 100.0f, r, g, b);
  
  enc.setRGB(r, g, b);

  Serial.print(h);
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
