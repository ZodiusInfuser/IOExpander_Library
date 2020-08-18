#include "IOExpander.h"

const uint8_t IOExpander::BIT_ADDRESSED_REGS[NUM_BIT_ADDRESSED_REGISTERS] = {REG_P0, REG_P1, REG_P2, REG_P3};
const uint8_t IOExpander::ENC_CFG[4] = {REG_ENC_1_CFG, REG_ENC_2_CFG, REG_ENC_3_CFG, REG_ENC_4_CFG};
const uint8_t IOExpander::ENC_COUNT[4] = {REG_ENC_1_COUNT, REG_ENC_2_COUNT, REG_ENC_3_COUNT, REG_ENC_4_COUNT};

const uint8_t IOExpander::Pin::PxM1[4] = {REG_P0M1, REG_P1M1, (uint8_t)-1, REG_P3M1};
const uint8_t IOExpander::Pin::PxM2[4] = {REG_P0M2, REG_P1M2, (uint8_t)-1, REG_P3M2};
const uint8_t IOExpander::Pin::Px[4] = {REG_P0, REG_P1, (uint8_t)-1, REG_P3};

const uint8_t IOExpander::Pin::PxS[4] = {REG_P0S, REG_P1S, (uint8_t)-1, REG_P3S};
const uint8_t IOExpander::Pin::MASK_P[4] = {REG_INT_MASK_P0, REG_INT_MASK_P1, (uint8_t)-1, REG_INT_MASK_P3};

const uint8_t IOExpander::Pin::PWML[6] = {REG_PWM0L, REG_PWM1L, REG_PWM2L, REG_PWM3L, REG_PWM4L, REG_PWM5L};
const uint8_t IOExpander::Pin::PWMH[6] = {REG_PWM0H, REG_PWM1H, REG_PWM2H, REG_PWM3H, REG_PWM4H, REG_PWM5H};

const char* IOExpander::MODE_NAMES[3] = {"IO", "PWM", "ADC"};
const char* IOExpander::GPIO_NAMES[4] = {"QB", "PP", "IN", "OD"};
const char* IOExpander::STATE_NAMES[2] = {"LOW", "HIGH"};
  

////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS/DESTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin::Pin(uint8_t port, uint8_t pin)
: _type(TYPE_IO)
, _mode(0)
, _port(port)
, _pin(pin)
, _adcChannel(0)
, _pwmChannel(0)
, _reg_m1(PxM1[port])
, _reg_m2(PxM2[port])
, _reg_p(Px[port])
, _reg_ps(PxS[port])
, _reg_int_mask_p(MASK_P[port])
, _regIoPwm(0)
, _reg_pwml(0)
, _reg_pwmh(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin::Pin(uint8_t port, uint8_t pin, uint8_t pwmChannel, uint8_t regIoPwm)
: _type(TYPE_PWM)
, _mode(0)
, _port(port)
, _pin(pin)
, _adcChannel(0)
, _pwmChannel(pwmChannel)
, _reg_m1(PxM1[port])
, _reg_m2(PxM2[port])
, _reg_p(Px[port])
, _reg_ps(PxS[port])
, _reg_int_mask_p(MASK_P[port])
, _regIoPwm(regIoPwm)
, _reg_pwml(PWML[pwmChannel])
, _reg_pwmh(PWMH[pwmChannel])
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin::Pin(uint8_t port, uint8_t pin, uint8_t adcChannel)
: _type(TYPE_ADC)
, _mode(0)
, _port(port)
, _pin(pin)
, _adcChannel(adcChannel)
, _pwmChannel(0)
, _reg_m1(PxM1[port])
, _reg_m2(PxM2[port])
, _reg_p(Px[port])
, _reg_ps(PxS[port])
, _reg_int_mask_p(MASK_P[port])
, _regIoPwm(0)
, _reg_pwml(0)
, _reg_pwmh(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin::Pin(uint8_t port, uint8_t pin, uint8_t adcChannel, uint8_t pwmChannel, uint8_t regIoPwm)
: _type(TYPE_ADC_OR_PWM)
, _mode(0)
, _port(port)
, _pin(pin)
, _adcChannel(adcChannel)
, _pwmChannel(pwmChannel)
, _reg_m1(PxM1[port])
, _reg_m2(PxM2[port])
, _reg_p(Px[port])
, _reg_ps(PxS[port])
, _reg_int_mask_p(MASK_P[port])
, _regIoPwm(regIoPwm)
, _reg_pwml(PWML[pwmChannel])
, _reg_pwmh(PWMH[pwmChannel])
{
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin IOExpander::Pin::Io(uint8_t port, uint8_t pin)
{
  return Pin(port, pin);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin IOExpander::Pin::Pwm(uint8_t port, uint8_t pin, uint8_t channel, uint8_t reg_iopwm)
{
  return Pin(port, pin, channel, reg_iopwm);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin IOExpander::Pin::Adc(uint8_t port, uint8_t pin, uint8_t channel)
{
  return Pin(port, pin, channel);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin IOExpander::Pin::AdcOrPwm(uint8_t port, uint8_t pin, uint8_t adc_channel, uint8_t pwm_channel, uint8_t reg_iopwm)
{
  return Pin(port, pin, adc_channel, pwm_channel, reg_iopwm);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool IOExpander::Pin::modeSupported(uint8_t mode)
{
  bool supported = false;
  if((_type & TYPE_PWM) && (mode == PIN_MODE_PWM))
  {
    supported = true;
  }
  else if((_type & TYPE_ADC) && (mode == PIN_MODE_ADC))
  {
    supported = true;
  }
  return supported;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::Pin::IOType IOExpander::Pin::getType(void)
{
  return _type;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t IOExpander::Pin::getMode(void)
{
  return _mode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::Pin::setMode(uint8_t mode)
{
  _mode = mode;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS/DESTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////
IOExpander::IOExpander(TwoWire& wire, uint8_t address, uint32_t timeout,
                       int8_t interruptPin, bool debug)
: _i2cAddr(address)
, _wire(wire)
, _debug(debug)
, _vref(3.3f)
, _timeout(timeout)
, _interruptPin(interruptPin)
, _encoderOffset{0,0,0,0}
, _encoderLast{0,0,0,0}
, _pins{Pin::Pwm(1, 5, 5, REG_PIOCON1),            
        Pin::Pwm(1, 0, 2, REG_PIOCON0),
        Pin::Pwm(1, 2, 0, REG_PIOCON0),
        Pin::Pwm(1, 4, 1, REG_PIOCON0),
        Pin::Pwm(0, 0, 3, REG_PIOCON0),
        Pin::Pwm(0, 1, 4, REG_PIOCON0),
        Pin::AdcOrPwm(1, 1, 7, 1, REG_PIOCON0),
        Pin::AdcOrPwm(0, 3, 6, 5, REG_PIOCON0),
        Pin::AdcOrPwm(0, 4, 5, 3, REG_PIOCON1),
        Pin::Adc(3, 0, 1),
        Pin::Adc(0, 6, 3),
        Pin::AdcOrPwm(0, 5, 4, 2, REG_PIOCON1),
        Pin::Adc(0, 7, 2),
        Pin::Adc(1, 7, 0)}
{  
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////
bool IOExpander::initialise(bool skipChipIdCheck)
{
  bool bSucceeded = true;
  
  if(_interruptPin != -1)
  {
    pinMode(_interruptPin, INPUT);
    enableInterruptOut();
  }

  if(!skipChipIdCheck)
  {
    uint16_t chipId = getChipId();
    if(chipId != CHIP_ID)
    {
      if(_debug)
      {
        Serial.print("Chip ID invalid: ");
        Serial.print(chipId, HEX);
        Serial.print(" expected: ");
        Serial.println(CHIP_ID, HEX);
      }
      bSucceeded = false;
    }
  }

  return bSucceeded;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t IOExpander::getChipId(void)
{
  //Get the IOE chip ID.
  return ((uint16_t)i2cRead8(REG_CHIP_ID_H) << 8) | (uint16_t)i2cRead8(REG_CHIP_ID_L);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setAddr(uint8_t i2cAddr)
{
  //Set the IOE i2c address.
  setBit(REG_CTRL, 4);
  i2cWrite8(REG_ADDR, i2cAddr);
  _i2cAddr = i2cAddr;
  delay(250); //TODO Handle addr change IOError better
  //waitForFlash()
  clrBit(REG_CTRL, 4);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float IOExpander::getAdcVref(void)
{
  //Get the ADC voltage reference.
  return _vref;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setAdcVref(float vref)
{
  //Set the ADC voltage reference.
  _vref = vref;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::enableInterruptOut(bool pinSwap)
{
  //Enable the IOE interrupts.
  setBit(REG_INT, BIT_INT_OUT_EN);
  changeBit(REG_INT, BIT_INT_PIN_SWAP, pinSwap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::disableInterruptOut(void)
{
  //Disable the IOE interrupt output.
  clrBit(REG_INT, BIT_INT_OUT_EN);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t IOExpander::getInterruptFlag(void)
{
  //Get the IOE interrupt flag.
  if(_interruptPin != 0)
    return digitalRead(_interruptPin) == 0;
  else
    return getBit(REG_INT, BIT_INT_TRIGD);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::clearInterruptFlag(void)
{
  //Clear the interrupt flag.
  clrBit(REG_INT, BIT_INT_TRIGD);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool IOExpander::setPinInterrupt(uint8_t pin, bool enabled)
{
  //Enable/disable the input interrupt on a specific pin.

  //param pin: Pin from 1-14
  //param enabled: True/False for enabled/disabled

  bool bSucceeded = false;
  if(pin >= 1 && pin <= NUM_PINS)
  {
    Pin& ioPin = _pins[pin - 1];
    changeBit(ioPin._reg_int_mask_p, ioPin._pin, enabled);
    
    bSucceeded = true;
  }
  
  return bSucceeded;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setInterruptCallback(void (*callback)())
{
  //Attach an event handler to be run on interrupt.
  
  //param callback: Callback function to run: callback()

  if(_interruptPin != 0 && callback != nullptr)
  {
    attachInterrupt(digitalPinToInterrupt(_interruptPin), callback, FALLING);
    clearInterruptFlag();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::pwmLoad(bool waitForLoad)
{
  //Load new period and duty registers into buffer
  unsigned long startTime = millis();
  setBit(REG_PWMCON0, 6);  //Set the "LOAD" bit of PWMCON0
  if(waitForLoad)
  {
    while(pwmLoading())
    {
      delay(1); //Wait for "LOAD" to complete
      if(millis() - startTime >= _timeout)
      {
        if(_debug)
          Serial.println("Timed out waiting for PWM load!");
        return;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool IOExpander::pwmLoading(void)
{
  return getBit(REG_PWMCON0, 6);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::pwmClear(bool waitForClear)
{
  //Clear the PWM counter
  unsigned long startTime = millis();
  setBit(REG_PWMCON0, 4);  //Set the "CLRPWM" bit of PWMCON0
  if(waitForClear)
  {
    while(pwmClearing())
    {
      delay(1); //Wait for "CLRPWM" to complete
      if(millis() - startTime >= _timeout)
      {
        if(_debug)
          Serial.println("Timed out waiting for PWM clear!");
        return;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool IOExpander::pwmClearing(void)
{
  return getBit(REG_PWMCON0, 4);
}
  
////////////////////////////////////////////////////////////////////////////////////////////////////
bool IOExpander::setPwmControl(uint8_t divider)
{
  //Set PWM settings.
  
  //PWM is driven by the 24MHz FSYS clock by default.
  
  //param divider: Clock divider, one of 1, 2, 4, 8, 16, 32, 64 or 128

  bool dividerGood = true;
  uint8_t pwmdiv2 = 0;
  switch(divider)
  {
    case 1:   pwmdiv2 = 0b000;    break;
    case 2:   pwmdiv2 = 0b001;    break;
    case 4:   pwmdiv2 = 0b010;    break;
    case 8:   pwmdiv2 = 0b011;    break;
    case 16:  pwmdiv2 = 0b100;    break;
    case 32:  pwmdiv2 = 0b101;    break;
    case 64:  pwmdiv2 = 0b110;    break;
    case 128: pwmdiv2 = 0b111;    break;

    default:
      if(_debug)
      {
        Serial.print("ValueError: A clock divider of ");
        Serial.println(divider);
      }
      dividerGood = false;
      break;
  }

  if(dividerGood)
  {
    //TODO: This currently sets GP, PWMTYP and FBINEN to 0
    //It might be desirable to make these available to the user
    //GP - Group mode enable (changes first three pairs of pAM to PWM01H and PWM01L)
    //PWMTYP - PWM type select: 0 edge-aligned, 1 center-aligned
    //FBINEN - Fault-break input enable

    i2cWrite8(REG_PWMCON1, pwmdiv2);
  }

  return dividerGood;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setPwmPeriod(uint16_t value, bool load)
{
  //Set the PWM period.
   
  //The period is the point at which the PWM counter is reset to zero.
  //The PWM clock runs at FSYS with a divider of 1/1.
  //Also specifies the maximum value that can be set in the PWM duty cycle.

  value &= 0xffff;
  i2cWrite8(REG_PWMPL, (uint8_t)(value & 0xff));
  i2cWrite8(REG_PWMPH, (uint8_t)(value >> 8));

  if(load)
    pwmLoad();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t IOExpander::getMode(uint8_t pin)
{
  //Get the current mode of a pin.
  return _pins[pin - 1].getMode();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setMode(uint8_t pin, uint8_t mode, bool schmittTrigger, bool invert)
{
  //Set a pin output mode.
  
  //param mode: one of the supplied IN, OUT, PWM or ADC constants

  if(pin < 1 || pin > NUM_PINS)
  {
    Serial.println("ValueError: Pin should be in range 1-14.");
    return;
  }

  Pin& ioPin = _pins[pin - 1];
  
  uint8_t gpioMode = mode & 0b11;
  uint8_t ioType = (mode >> 2) & 0b11;
  uint8_t initialState = mode >> 4;
  
  if(ioPin.getMode() == mode)
  {
    if(_debug)
    {
      Serial.print("Mode already is ");
      Serial.println(MODE_NAMES[ioType]);
    }
    return;
  }

  if((ioType != Pin::TYPE_IO) && !ioPin.modeSupported(mode))
  {
    if(_debug)
    {
      Serial.print("Pin ");
      Serial.print(pin);
      Serial.print(" does not support ");
      Serial.print(MODE_NAMES[ioType]);
      Serial.println("!");
    }
    return;
  }

  ioPin.setMode(mode);
  if(_debug)
  {
    Serial.print("Setting pin ");
    Serial.print(pin);
    Serial.print(" to mode ");
    Serial.print(MODE_NAMES[ioType]);
    Serial.print(" ");
    Serial.print(GPIO_NAMES[gpioMode]);
    Serial.print(", state: ");
    Serial.print(STATE_NAMES[initialState]);
  }

  if(mode == PIN_MODE_PWM)
  {
    setBit(ioPin._regIoPwm, ioPin._pwmChannel);
    changeBit(REG_PNP, ioPin._pwmChannel, invert);
    setBit(REG_PWMCON0, 7);  //Set PWMRUN bit
  }
  else
  {
    if(ioPin.getType() & Pin::TYPE_PWM)
      clrBit(ioPin._regIoPwm, ioPin._pwmChannel);
  }

  uint8_t pm1 = i2cRead8(ioPin._reg_m1);
  uint8_t pm2 = i2cRead8(ioPin._reg_m2);

  //Clear the pm1 and pm2 bits
  pm1 &= 255 - (1 << ioPin._pin);
  pm2 &= 255 - (1 << ioPin._pin);

  //Set the new pm1 and pm2 bits according to our gpio_mode
  pm1 |= (gpioMode >> 1) << ioPin._pin;
  pm2 |= (gpioMode & 0b1) << ioPin._pin;

  i2cWrite8(ioPin._reg_m1, pm1);
  i2cWrite8(ioPin._reg_m2, pm2);

  //Set up Schmitt trigger mode on inputs
  if(mode == PIN_MODE_PU || mode == PIN_MODE_IN)
    changeBit(ioPin._reg_ps, ioPin._pin, schmittTrigger);

  //5th bit of mode encodes default output pin state
  i2cWrite8(ioPin._reg_p, (initialState << 3) | ioPin._pin);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int16_t IOExpander::input(uint8_t pin, uint32_t adcTimeout)
{
  //Read the IO pin state.
  
  //Returns a 12-bit ADC reading if the pin is in ADC mode
  //Returns True/False if the pin is in any other input mode
  //Returns None if the pin is in PWM mode
  //:param adcTimeout: Timeout (in seconds) for an ADC read (default 1.0)

  if(pin < 1 || pin > NUM_PINS)
  {
    if(_debug)
      Serial.println("ValueError: Pin should be in range 1-14.");
    return -1;
  }

  Pin& ioPin = _pins[pin - 1];

  if(ioPin.getMode() == PIN_MODE_ADC)
  {
    if(_debug)
    {
      Serial.print("Reading ADC from pin ");
      Serial.println(pin);
    }
      
    clrBits(REG_ADCCON0, 0x0f);
    setBits(REG_ADCCON0, ioPin._adcChannel);
    i2cWrite8(REG_AINDIDS, 0);
    setBit(REG_AINDIDS, ioPin._adcChannel);
    setBit(REG_ADCCON1, 0);

    clrBit(REG_ADCCON0, 7);  //ADCF - Clear the conversion complete flag
    setBit(REG_ADCCON0, 6);  //ADCS - Set the ADC conversion start flag

    //Wait for the ADCF conversion complete flag to be set
    unsigned long startTime = millis();
    while(!getBit(REG_ADCCON0, 7))
    {
      delay(10);
      if(millis() - startTime >= adcTimeout)
      {
        if(_debug)
          Serial.println("Timeout waiting for ADC conversion!");
        return -1;
      }
    }

    uint8_t hi = i2cRead8(REG_ADCRH);
    uint8_t lo = i2cRead8(REG_ADCRL);
    return (uint16_t)(hi << 4) | (uint16_t)lo;
  }
  else
  {
    if(_debug)
    {
      Serial.print("Reading IO from pin ");
      Serial.println(pin);
    }
    
    uint8_t pv = getBit(ioPin._reg_p, ioPin._pin);
    return (pv) ? HIGH : LOW;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
float IOExpander::inputAsVoltage(uint8_t pin, uint32_t adcTimeout)
{
  //Read the IO pin state.
  
  //Returns a 12-bit ADC reading if the pin is in ADC mode
  //Returns True/False if the pin is in any other input mode
  //Returns None if the pin is in PWM mode
  //:param adcTimeout: Timeout (in seconds) for an ADC read (default 1.0)

  if(pin < 1 || pin > NUM_PINS)
  {
    if(_debug)
      Serial.println("ValueError: Pin should be in range 1-14.");
    return -1;
  }

  Pin& ioPin = _pins[pin - 1];

  if(ioPin.getMode() == PIN_MODE_ADC)
  {
    if(_debug)
    {
      Serial.print("Reading ADC from pin ");
      Serial.println(pin);
    }
      
    clrBits(REG_ADCCON0, 0x0f);
    setBits(REG_ADCCON0, ioPin._adcChannel);
    i2cWrite8(REG_AINDIDS, 0);
    setBit(REG_AINDIDS, ioPin._adcChannel);
    setBit(REG_ADCCON1, 0);

    clrBit(REG_ADCCON0, 7);  //ADCF - Clear the conversion complete flag
    setBit(REG_ADCCON0, 6);  //ADCS - Set the ADC conversion start flag

    //Wait for the ADCF conversion complete flag to be set
    unsigned long startTime = millis();
    while(!getBit(REG_ADCCON0, 7))
    {
      delay(1);
      if(millis() - startTime >= adcTimeout)
      {
        if(_debug)
          Serial.println("Timeout waiting for ADC conversion!");
        return -1;
      }
    }

    uint8_t hi = i2cRead8(REG_ADCRH);
    uint8_t lo = i2cRead8(REG_ADCRL);
    return ((float)((uint16_t)(hi << 4) | (uint16_t)lo) / 4095.0f) * _vref;
  }
  else
  {
    if(_debug)
    {
      Serial.print("Reading IO from pin ");
      Serial.println(pin);
    }
    
    uint8_t pv = getBit(ioPin._reg_p, ioPin._pin);
    return (pv) ? _vref : 0.0f;
  }
}
  
////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::output(uint8_t pin, uint16_t value, bool load)
{
  //Write an IO pin state or PWM duty cycle.
  
  //param value: Either True/False for OUT, or a number between 0 and PWM period for PWM.
  
  if(pin < 1 || pin > NUM_PINS)
  {
    Serial.println("Pin should be in range 1-14.");
    return;
  }

  Pin& ioPin = _pins[pin - 1];

  if(ioPin.getMode() == PIN_MODE_PWM)
  {
    if(_debug)
    {
      Serial.print("Outputting PWM to pin: ");
      Serial.println("pin");
    }

    i2cWrite8(ioPin._reg_pwml, (uint8_t)(value & 0xff));
    i2cWrite8(ioPin._reg_pwmh, (uint8_t)(value >> 8));
    if(load)
      pwmLoad();
  }
  else
  {
    if(value == LOW)
    {
      if(_debug)
      {
        Serial.print("Outputting LOW to pin: ");
        Serial.println(pin);
      }
      
      clrBit(ioPin._reg_p, ioPin._pin);
    }
    else if(value == HIGH)
    {
      if(_debug)
      {
        Serial.print("Outputting HIGH to pin: ");
        Serial.println(pin);
      }
      
      setBit(ioPin._reg_p, ioPin._pin);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setupRotaryEncoder(uint8_t channel, uint8_t pinA, uint8_t pinB, uint8_t pinC, bool countMicrosteps)
{
  //Set up a rotary encoder.
  channel -= 1;
  setMode(pinA, PIN_MODE_PU, true);
  setMode(pinB, PIN_MODE_PU, true);

  if(pinC != 0)
  {
    setMode(pinC, PIN_MODE_OD);
    output(pinC, 0);
  }

  i2cWrite8(ENC_CFG[channel], pinA | (pinB << 4));
  changeBit(REG_ENC_EN, (channel * 2) + 1, countMicrosteps);
  setBit(REG_ENC_EN, channel * 2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t IOExpander::readRotaryEncoder(uint8_t channel)
{
  //Read the step count from a rotary encoder."""
  channel -= 1;
  uint8_t last = _encoderLast[channel];
  uint8_t reg = ENC_COUNT[channel];
  uint8_t value = i2cRead8(reg);

  if(value & 0b10000000)
    value -= 256;

  if(last > 64 && value < -64)
    _encoderOffset[channel] += 256;
  if(last < -64 && value > 64)
    _encoderOffset[channel] -= 256;

  _encoderLast[channel] = value;

  return _encoderOffset[channel] + value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t IOExpander::i2cRead8(uint8_t reg)
{
  //Read a single (8bit) register from the device.
  _wire.beginTransmission(_i2cAddr);
  _wire.write(reg);
  _wire.endTransmission();
  _wire.requestFrom((int)_i2cAddr, 1); 
  uint8_t value = _wire.read();

  return value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::i2cWrite8(uint8_t reg, uint8_t value)
{
  //Write a single (8bit) register to the device.
  _wire.beginTransmission(_i2cAddr);
  _wire.write(reg);
  _wire.write(value);
  _wire.endTransmission();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t IOExpander::getBit(uint8_t reg, uint8_t bit)
{
  //Returns the specified bit (nth position from right) from a register
  return i2cRead8(reg) & (1 << bit);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setBits(uint8_t reg, uint8_t bits)
{
  //Set the specified bits (using a mask) in a register.

  //Deal with special case registers first
  bool regInBitAddressedRegs = false;
  for(uint8_t i = 0; i < NUM_BIT_ADDRESSED_REGISTERS; i++)
  {
    if(BIT_ADDRESSED_REGS[i] == reg)
    {
      for(uint8_t bit = 0; bit < 8; bit++)
      {
        if(bits & (1 << bit))
          i2cWrite8(reg, 0b1000 | (bit & 0b111));
      }
      regInBitAddressedRegs = true;
      break;
    }
  }

  //Now deal with any other registers
  if(!regInBitAddressedRegs)
  {
    uint8_t value = i2cRead8(reg);
    delay(1);
    i2cWrite8(reg, value | bits);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::setBit(uint8_t reg, uint8_t bit)
{
  //Set the specified bit (nth position from right) in a register.
  setBits(reg, (1 << bit));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::clrBits(uint8_t reg, uint8_t bits)
{
  //Clear the specified bits (using a mask) in a register.

  //Deal with special case registers first
  bool regInBitAddressedRegs = false;
  for(uint8_t i = 0; i < NUM_BIT_ADDRESSED_REGISTERS; i++)
  {
    if(BIT_ADDRESSED_REGS[i] == reg)
    {
      for(uint8_t bit = 0; bit < 8; bit++)
      {
        if(bits & (1 << bit))
          i2cWrite8(reg, 0b0000 | (bit & 0b111));
      }
      regInBitAddressedRegs = true;
      break;
    }
  }

  //Now deal with any other registers
  if(!regInBitAddressedRegs)
  {
    uint8_t value = i2cRead8(reg);
    delay(1);
    i2cWrite8(reg, value & ~bits);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::clrBit(uint8_t reg, uint8_t bit)
{
  //Clear the specified bit (nth position from right) in a register.
  clrBits(reg, (1 << bit));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::changeBit(uint8_t reg, uint8_t bit, bool state)
{
  //Toggle one register bit on/off.
  if(state)
    setBit(reg, bit);
  else
    clrBit(reg, bit);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void IOExpander::waitForFlash(void)
{
  //Wait for the IOE to finish writing non-volatile memory.
  unsigned long startTime = millis();
  while(getInterruptFlag())
  {
    if(millis() - startTime > _timeout)
    {
      Serial.println("Timed out waiting for interrupt!");
      return;
    }
    delay(1);
  }

  startTime = millis();
  while(!getInterruptFlag())
  {
    if(millis() - startTime > _timeout)
    {
      Serial.println("Timed out waiting for interrupt!");
      return;
    }
    delay(1);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
