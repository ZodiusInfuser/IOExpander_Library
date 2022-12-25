#ifndef _GASBREAKOUT_h
#define _GASBREAKOUT_h

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
class GasBreakout
{
  //--------------------------------------------------
  // Constants
  //--------------------------------------------------
public:
  static const uint8_t DEFAULT_I2C_ADDRESS = 0x19;
  static constexpr float DEFAULT_BRIGHTNESS = 1.0f; //Effectively the maximum fraction of the period that the LED will be on
  static const uint32_t DEFAULT_ADC_TIMEOUT = 1;
private:
  static const byte PIN_RED = 3;
  static const byte PIN_GREEN = 7;
  static const byte PIN_BLUE = 2;

  static const byte MICS6814_VREF = 14;
  static const byte MICS6814_RED = 13;
  static const byte MICS6814_NH3 = 11;
  static const byte MICS6814_OX = 12;

  static const byte PIN_HEATER_EN = 1;

  static const bool INVERT_OUTPUT = true; //true for common cathode, false for common anode

    //--------------------------------------------------
    // Substructures
    //--------------------------------------------------
public:
  struct Reading {
    float ref;
    float reducing;
    float nh3;
    float oxidising;
  };
  //--------------------------------------------------
  // Constructors/Destructor
  //--------------------------------------------------
public:
  GasBreakout(TwoWire& wire, uint8_t address = DEFAULT_I2C_ADDRESS, float brightness = DEFAULT_BRIGHTNESS,
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
  
  //--------------------------------------------------
  //MIC6814 Gas Breakout specific
  //--------------------------------------------------
  
  void setBrightness(float brightness);
  void setRGB(uint8_t r, uint8_t g, uint8_t b);
  void setHeater(bool value);

  Reading readAll(uint32_t adcTimeout = DEFAULT_ADC_TIMEOUT);
  float readReducing(uint32_t adcTimeout = DEFAULT_ADC_TIMEOUT);
  float readNH3(uint32_t adcTimeout = DEFAULT_ADC_TIMEOUT);
  float readOxidising(uint32_t adcTimeout = DEFAULT_ADC_TIMEOUT);
  float readRef(uint32_t adcTimeout = DEFAULT_ADC_TIMEOUT);
private:
  float readGas(byte channel, uint32_t adcTimeout = DEFAULT_ADC_TIMEOUT);
  //--------------------------------------------------
  // Variables
  //--------------------------------------------------
private:
  IOExpander _ioe;
  float _brightness;
};
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
