#ifndef _POTBREAKOUT_h
#define _POTBREAKOUT_h

/***** Includes *****/
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Wire.h"
#include "IOExpander.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
class PotBreakout
{
  //--------------------------------------------------
  // Constants
  //--------------------------------------------------
public:
  static const uint8_t DEFAULT_I2C_ADDRESS = 0x0E;
  static constexpr float DEFAULT_BRIGHTNESS = 1.0f; //Effectively the maximum fraction of the period that the LED will be on
private:
  static const byte PIN_RED = 1;
  static const byte PIN_GREEN = 7;
  static const byte PIN_BLUE = 2;

  static const byte PIN_TERM_A = 12;
  static const byte PIN_TERM_B = 3;
  static const byte PIN_INPUT = 11;

  static const bool INVERT_OUTPUT = true; //true for common cathode, false for common anode


  //--------------------------------------------------
  // Constructors/Destructor
  //--------------------------------------------------
public:
  PotBreakout(TwoWire& wire, uint8_t address = DEFAULT_I2C_ADDRESS, float brightness = DEFAULT_BRIGHTNESS,
              uint32_t timeout = 1, int8_t interruptPin = -1, bool debug = false);


  //--------------------------------------------------
  // Methods
  //--------------------------------------------------
public:
  bool initialise(bool skipChipIdCheck = false);

  //--------------------------------------------------
  //Calls through to IOExpander class
  //--------------------------------------------------
  void setAddr(uint8_t i2cAddr);
  
  float getAdcVref(void);
  void setAdcVref(float vref);

  //--------------------------------------------------
  //Potentiometer Breakout specific
  //--------------------------------------------------
  
  bool getDirection(void);  
  void setDirection(bool clockwise);
  
  void setBrightness(float brightness);
  void setRGB(uint8_t r, uint8_t g, uint8_t b);

  int16_t read(uint32_t adcTimeout = 1);
  float readAsPercent(uint32_t adcTimeout = 1);


  //--------------------------------------------------
  // Variables
  //--------------------------------------------------
private:
  IOExpander _ioe;
  bool _direction;
  float _brightness;
};
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
