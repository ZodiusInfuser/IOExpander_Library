#include "EncBreakout.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS/DESTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////
EncBreakout::EncBreakout(TwoWire& wire, uint8_t address, float brightness, uint32_t timeout, int8_t interruptPin, bool debug)
: _ioe(wire, address, timeout, interruptPin, debug)
, _direction(true) //clockwise
, _brightness(brightness)
, _interruptPin(interruptPin)
{    
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////
bool EncBreakout::initialise(bool skipChipIdCheck)
{
  bool bSucceeded = true;
  if(_ioe.initialise(skipChipIdCheck))
  {
    if(_interruptPin != -1)
      _ioe.enableInterruptOut(true);
      
    _ioe.setupRotaryEncoder(ENC_CHANNEL, PIN_ENC_A, PIN_ENC_B, PIN_ENC_C);

    //Calculate a period large enough to get 0-255 steps at the desired brightness
    unsigned int period = (unsigned int)(255.0f / _brightness);
  
    _ioe.setPwmPeriod(period);
    _ioe.setPwmControl(2);  //PWM as fast as we can to avoid LED flicker

    _ioe.setMode(PIN_RED, IOExpander::PIN_PWM, false, INVERT_OUTPUT);
    _ioe.setMode(PIN_GREEN, IOExpander::PIN_PWM, false, INVERT_OUTPUT);
    _ioe.setMode(PIN_BLUE, IOExpander::PIN_PWM, false, INVERT_OUTPUT);
    
    bSucceeded = true;
  }
  return bSucceeded;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void EncBreakout::setAddr(uint8_t i2cAddr)
{
  _ioe.setAddr(i2cAddr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void EncBreakout::setInterruptCallback(void (*callback)())
{
  _ioe.setInterruptCallback(callback);
}
  
////////////////////////////////////////////////////////////////////////////////////////////////////
bool EncBreakout::getDirection(void)
{
  return _direction;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void EncBreakout::setDirection(bool clockwise)
{
  _direction = clockwise;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void EncBreakout::setBrightness(float brightness)
{
  _brightness = constrain(brightness, 0.01f, 1.0f);
  
  //Calculate a period large enough to get 0-255 steps at the desired brightness
  unsigned int period = (unsigned int)(255.0f / _brightness);
  
  _ioe.setPwmPeriod(period);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void EncBreakout::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
  _ioe.output(PIN_RED, r, false);   //Hold off pwm load until the last
  _ioe.output(PIN_GREEN, g, false); //Hold off pwm load until the last
  _ioe.output(PIN_BLUE, b);         //Loads all 3 pwms
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool EncBreakout::available(void)
{
  return (_ioe.getInterruptFlag() > 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t EncBreakout::read(void)
{
  int16_t count = _ioe.readRotaryEncoder(ENC_CHANNEL);
  if(!_direction)
    count = 0 - count;
    
  _ioe.clearInterruptFlag();
  return count;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
