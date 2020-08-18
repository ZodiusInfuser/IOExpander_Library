/*
 * IOExpander servo example (updated August 18, 2020)
 * 
 * Demonstrates running a servo at 50Hz
 * 
 * Ported from https://github.com/pimoroni/ioe-python/blob/master/examples/servo.py
 * by Christopher (@ZodiusInfuser) Parrott
 */

/***** Library Includes *****/
#include <IOExpander.h>

/***** Global Constants *****/
static const unsigned int DELAY_MS = 1000 / 60;

static const byte PIN_SERVO = 1;

//Settings to produce a 50Hz output from the 24MHz clock.
//24,000,000 Hz / 8 = 3,000,000 Hz
//3,000,000 Hz / 60,000 Period = 50 Hz
static const byte DIVIDER = 8;
static const unsigned int PERIOD = 60000;

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
  ioe.setPwmControl(DIVIDER);

  ioe.setMode(PIN_SERVO, IOExpander::PIN_PWM);
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP
////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  float t = (float)millis() / 10000.0f;
  float s = (sinf(t * PI) + 1.0f) / 2.0f;
  float servoUS= 1000.0f + (s * 1000.0f);   //Between 1000 and 2000us (1-2ms)

  float dutyPerMicrosecond = (float)PERIOD / (float)(20 * 1000);  //Default is 3 LSB per microsecond

  unsigned int dutyCycle = (unsigned int)(roundf(servoUS * dutyPerMicrosecond));
  Serial.print("Pulse ");
  Serial.print(servoUS);
  Serial.print("us, Duty Cycle: ");
  Serial.println(dutyCycle);

  ioe.output(PIN_SERVO, dutyCycle);

  delay(DELAY_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
