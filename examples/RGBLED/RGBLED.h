#ifndef _RGBLED_h
#define _RGBLED_h

/***** Includes *****/
#include <IOExpander.h>


////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
class RGBLED
{
  //--------------------------------------------------
  // Constructors/Destructor
  //--------------------------------------------------
public:
  RGBLED(IOExpander& ioe, byte pinR = 1, byte pinG = 3, byte pinB = 5, float brightness = 0.05f, bool invert = false, bool debug = false);


  //--------------------------------------------------
  // Methods
  //--------------------------------------------------
public:
  bool initialise(void);
  
  void setBrightness(float brightness);
  void setRGB(byte r, byte g, byte b);
  void setHue(float h, float s = 100.0f, float v = 100.0f);

private:
  void HSVtoRGB(float hue, float sat, float val, byte& rOut, byte& gOut, byte& bOut);
   

  //--------------------------------------------------
  // Variables
  //--------------------------------------------------
private:
   IOExpander& _ioe;
   byte _pinR;
   byte _pinG;
   byte _pinB;

   unsigned int _period;
   float _brightness;
   bool _invert;
   bool _debug;
};
#endif
