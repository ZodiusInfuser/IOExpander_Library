#ifndef _ENCBREAKOUT_h
#define _ENCBREAKOUT_h

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
class EncBreakout
{
  //--------------------------------------------------
  // Constants
  //--------------------------------------------------
public:
  static const uint8_t DEFAULT_I2C_ADDRESS = 0x0F;
  static constexpr float DEFAULT_BRIGHTNESS = 1.0f; //Effectively the maximum fraction of the period that the LED will be on
private:
  static const byte PIN_RED = 1;
  static const byte PIN_GREEN = 7;
  static const byte PIN_BLUE = 2;

  static const byte PIN_ENC_A = 12;
  static const byte PIN_ENC_B = 3;
  static const byte PIN_ENC_C = 11;

  static const byte ENC_CHANNEL = 1;

  static const bool INVERT_OUTPUT = true; //true for common cathode, false for common anode


  //--------------------------------------------------
  // Constructors/Destructor
  //--------------------------------------------------
public:
  EncBreakout(TwoWire& wire, uint8_t address = DEFAULT_I2C_ADDRESS, float brightness = DEFAULT_BRIGHTNESS,
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
  void setInterruptCallback(void (*callback)());
  

  //--------------------------------------------------
  //Potentiometer Breakout specific
  //--------------------------------------------------
  
  bool getDirection(void);  
  void setDirection(bool clockwise);
  
  void setBrightness(float brightness);
  void setRGB(uint8_t r, uint8_t g, uint8_t b);

  bool available(void);
  int16_t read(void);


  //--------------------------------------------------
  // Variables
  //--------------------------------------------------
public:
  IOExpander _ioe;
  bool _direction;
  float _brightness;
  int8_t _interruptPin;
};
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
