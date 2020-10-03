#include "PotBreakout.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS/DESTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////
PotBreakout::PotBreakout(TwoWire& wire, uint8_t address, float brightness, uint32_t timeout, int8_t interruptPin, bool debug)
: _ioe(wire, address, timeout, interruptPin, debug)
, _direction(true) //clockwise
, _brightness(brightness)
{    
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////
bool PotBreakout::initialise(bool skipChipIdCheck)
{
  bool bSucceeded = true;
  if(_ioe.initialise(skipChipIdCheck))
  {
    _ioe.setMode(PIN_TERM_A, IOExpander::PIN_OUT);
    _ioe.setMode(PIN_TERM_B, IOExpander::PIN_OUT);
    _ioe.setMode(PIN_INPUT, IOExpander::PIN_ADC);

    if(_direction)
    {
      //Clockwise increasing
      _ioe.output(PIN_TERM_A, LOW);
      _ioe.output(PIN_TERM_B, HIGH);
    }
    else
    {
      //Counter clockwise increasing
      _ioe.output(PIN_TERM_A, HIGH);
      _ioe.output(PIN_TERM_B, LOW);
    }

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
void PotBreakout::setAddr(uint8_t i2cAddr)
{
  _ioe.setAddr(i2cAddr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float PotBreakout::getAdcVref(void)
{
  return _ioe.getAdcVref();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PotBreakout::setAdcVref(float vref)
{
  _ioe.setAdcVref(vref);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool PotBreakout::getDirection(void)
{
  return _direction;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PotBreakout::setDirection(bool clockwise)
{
  if(clockwise)
  {
    //Clockwise increasing
    _ioe.output(PIN_TERM_A, LOW);
    _ioe.output(PIN_TERM_B, HIGH);
  }
  else
  {
    //Counter clockwise increasing
    _ioe.output(PIN_TERM_A, HIGH);
    _ioe.output(PIN_TERM_B, LOW);
  }
  _direction = clockwise;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PotBreakout::setBrightness(float brightness)
{
  _brightness = constrain(brightness, 0.01f, 1.0f);
  
  //Calculate a period large enough to get 0-255 steps at the desired brightness
  unsigned int period = (unsigned int)(255.0f / _brightness);
  
  _ioe.setPwmPeriod(period);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PotBreakout::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
  _ioe.output(PIN_RED, r, false);   //Hold off pwm load until the last
  _ioe.output(PIN_GREEN, g, false); //Hold off pwm load until the last
  _ioe.output(PIN_BLUE, b);         //Loads all 3 pwms
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t PotBreakout::read(uint32_t adcTimeout)
{
   return _ioe.input(PIN_INPUT, adcTimeout);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float PotBreakout::readAsPercent(uint32_t adcTimeout)
{
  return (_ioe.inputAsVoltage(PIN_INPUT, adcTimeout) / _ioe.getAdcVref());
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
