/*
 * IOExpander rotary example (updated October 3, 2020)
 * 
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/rotary.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

//Uncomment this to use the interrupt routine
//#define USE_INTERRUPT_ROUTINE

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
IOExpander ioe(Wire, 0x18, 1, 4);
int16_t count = 0;

#ifdef USE_INTERRUPT_ROUTINE
  volatile bool intHandled = false;
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_INTERRUPT_ROUTINE
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
  if(!ioe.initialise())
  {
    while(true)
      delay(10);
  }

  ioe.enableInterruptOut();
  #ifdef USE_INTERRUPT_ROUTINE
    ioe.setInterruptCallback(interruptRoutine);
  #endif

  ioe.setupRotaryEncoder(1, POT_ENC_A, POT_ENC_B, POT_ENC_C);

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
  #ifdef USE_INTERRUPT_ROUTINE
    if(intHandled)
    { 
      count = ioe.readRotaryEncoder(1);
      ioe.clearInterruptFlag();
      intHandled = false;
    }
  #else
    if(ioe.getInterruptFlag())
    { 
      count = ioe.readRotaryEncoder(1);
      ioe.clearInterruptFlag();
    }
  #endif   
  
  while(count < 0)
    count += 360;

  float h = (float)(count % 360);
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
