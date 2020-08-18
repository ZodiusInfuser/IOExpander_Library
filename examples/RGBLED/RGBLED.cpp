#include "RGBLED.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS/DESTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////
RGBLED::RGBLED(IOExpander& ioe, byte pinR, byte pinG, byte pinB, float brightness, bool invert, bool debug)
: _ioe(ioe)
, _pinR(pinR)
, _pinG(pinG)
, _pinB(pinB)
, _period((unsigned int)(255.0f / brightness))
, _brightness(brightness)
, _invert(invert)
, _debug(debug)
{
  //RGB LED Device.
  
  //IO Expander helper class to drive a single RGB LED connected to three PWM pins.
  //Note: Will set the global PWM period and divider.
  
  //param ioe: Instance of IOExpander
  //param pinR: Pin for the red LED
  //param pinG: Pin for the green LED
  //param pinB: Pin for the blue LED
  //param invert: Whether to invert the PWM signal (for common-cathode LEDs)
  //param brightness: LED brightness- translates to the maximum PWM duty cycle
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////
bool RGBLED::initialise(void)
{
  bool initialised = false;
  
  if(_ioe.initialise())
  {
    _ioe.setPwmPeriod(_period);
    _ioe.setPwmControl(1);
  
    _ioe.setMode(_pinR, IOExpander::PIN_PWM, false, _invert);
    _ioe.setMode(_pinG, IOExpander::PIN_PWM, false, _invert);
    _ioe.setMode(_pinB, IOExpander::PIN_PWM, false, _invert);
    
    initialised = true;
  }
  return initialised;
}
  
void RGBLED::setBrightness(float brightness)
{
  //Set the LED brightness.
  
  //Re-calculates the PWM period so that colour fidelity is not lost.
  
  //param brightness: LED brightness (0.0 to 1.0)
  
  _period = (unsigned int)(255.0f / brightness);
  _brightness = brightness;

  _ioe.setPwmPeriod(_period);
}

void RGBLED::setRGB(byte r, byte g, byte b)
{
  //Set the LED to an RGB colour.
  
  //param r: Red amount (0 to 255)
  //param g: Green amount (0 to 255)
  //param b: Blue amount (0 to 255)

  _ioe.output(_pinR, r);
  _ioe.output(_pinG, g);
  _ioe.output(_pinB, b);

  if(_debug)
  {
    Serial.print(r);
    Serial.print(", ");
    Serial.print(g);
    Serial.print(", ");
    Serial.println(b);
  }
}

void RGBLED::setHue(float h, float s, float v)
{
  //Set the LED to a hue.
  
  //param h: Hue (0 to 360.0)
  //param s: Saturation (0 to 100.0)
  //param v: Value (0 to 100.0)

  byte r, g, b;
  HSVtoRGB(h, s, v, r, g, b);
  setRGB(r, g, b);
}

void RGBLED::HSVtoRGB(float hue, float sat, float val, byte& rOut, byte& gOut, byte& bOut)
{
  ////////////////////////////////
  // Convert HSV values to RGB
  // H(Hue): 0-360 degrees
  // S(Saturation): 0-100 percent
  // V(Value): 0-100 percent
  //
  // RGB out is in range 0->255.
  // This method was found at:
  // https://www.codespeedy.com/hsv-to-rgb-in-cpp/
  ////////////////////////////////
  if(hue > 360.0f || hue < 0.0f || sat > 100.0f || sat < 0.0f || val > 100.0f || val < 0.0f)
  {
    //The given HSV values are not in valid range
    rOut = 0;
    gOut = 0;
    bOut = 0;
    return;
  }
  
  float s = sat / 100.0f;
  float v = val / 100.0f;
  float c = s * v;
  float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
  float m = v - c;
  
  float r, g, b;
  if(hue >= 0.0f && hue < 60.0f)
      r = c, g = x, b = 0.0f;
  else if(hue >= 60.0f && hue < 120.0f)
      r = x, g = c, b = 0.0f;
  else if(hue >= 120.0f && hue < 180.0f)
      r = 0.0f, g = c, b = x;
  else if(hue >= 180.0f && hue < 240.0f)
      r = 0.0f, g = x, b = c;
  else if(hue >= 240.0f && hue < 300.0f)
      r = x, g = 0.0f, b = c;
  else
      r = c, g = 0.0f, b = x;

  rOut = (byte)((r + m) * 255.0f);
  gOut = (byte)((g + m) * 255.0f);
  bOut = (byte)((b + m) * 255.0f);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
