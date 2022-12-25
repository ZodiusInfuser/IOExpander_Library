#include "GasBreakout.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS/DESTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////
GasBreakout::GasBreakout(TwoWire& wire, uint8_t address, float brightness, uint32_t timeout, int8_t interruptPin, bool debug)
: _ioe(wire, address, timeout, interruptPin, debug)
, _brightness(brightness)
{    
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////
bool GasBreakout::initialise(bool skipChipIdCheck)
{
  bool bSucceeded = false;
  if(_ioe.initialise(skipChipIdCheck))
  {
      
    _ioe.setMode(MICS6814_RED, IOExpander::PIN_ADC);
    _ioe.setMode(MICS6814_NH3, IOExpander::PIN_ADC);
    _ioe.setMode(MICS6814_OX, IOExpander::PIN_ADC);

    _ioe.setMode(PIN_HEATER_EN, IOExpander::PIN_OUT);
    _ioe.output(PIN_HEATER_EN, LOW);

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
void GasBreakout::setAddr(uint8_t i2cAddr)
{
  _ioe.setAddr(i2cAddr);
}
  
////////////////////////////////////////////////////////////////////////////////////////////////////
void GasBreakout::setBrightness(float brightness)
{
  _brightness = constrain(brightness, 0.01f, 1.0f);
  
  //Calculate a period large enough to get 0-255 steps at the desired brightness
  unsigned int period = (unsigned int)(255.0f / _brightness);
  
  _ioe.setPwmPeriod(period);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void GasBreakout::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
  _ioe.output(PIN_RED, r, false);   //Hold off pwm load until the last
  _ioe.output(PIN_GREEN, g, false); //Hold off pwm load until the last
  _ioe.output(PIN_BLUE, b);         //Loads all 3 pwms
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void GasBreakout::setHeater(bool value)
{
  uint16_t state = (value)?LOW:HIGH;

  _ioe.output(PIN_HEATER_EN, state);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float GasBreakout::readGas(byte channel, uint32_t adcTimeout)
{
  float vref = _ioe.getAdcVref();
  float vgas = _ioe.inputAsVoltage(channel, adcTimeout);
  float res = 0;

  if (vref != vgas)
    res = (vgas * 56000) / (vref - vgas);   

  return res;
}

GasBreakout::Reading GasBreakout::readAll(uint32_t adcTimeout)
{
  GasBreakout::Reading reading;
  reading.reducing = readReducing(adcTimeout);
  reading.nh3 = readNH3(adcTimeout);
  reading.oxidising = readOxidising(adcTimeout);
  reading.ref = readRef(adcTimeout);

  return reading;
}
float GasBreakout::readReducing(uint32_t adcTimeout)
{
  return readGas(MICS6814_RED, adcTimeout);
}
float GasBreakout::readNH3(uint32_t adcTimeout)
{
  return readGas(MICS6814_NH3, adcTimeout);
}
float GasBreakout::readOxidising(uint32_t adcTimeout)
{
  return readGas(MICS6814_OX, adcTimeout);
}
float GasBreakout::readRef(uint32_t adcTimeout)
{
  float ref = _ioe.inputAsVoltage(MICS6814_VREF, adcTimeout);

  if (ref == -1)
    ref = 0;

  return ref;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
