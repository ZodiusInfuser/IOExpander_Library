#ifndef _IOEXPANDER_h
#define _IOEXPANDER_h

/***** Includes *****/
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Wire.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Classes
////////////////////////////////////////////////////////////////////////////////////////////////////
class IOExpander
{
  //--------------------------------------------------
  // Constants
  //--------------------------------------------------
public:
  static const uint8_t DEFAULT_I2C_ADDRESS = 0x18;
  static const uint16_t CHIP_ID = 0xE26A;
  static const uint8_t CHIP_VERSION = 2;
private:
  static const uint8_t REG_CHIP_ID_L = 0xfa;
  static const uint8_t REG_CHIP_ID_H = 0xfb;
  static const uint8_t REG_VERSION = 0xfc;
  
  //Rotary encoder
  static const uint8_t REG_ENC_EN = 0x04;
  static const uint8_t BIT_ENC_EN_1 = 0;
  static const uint8_t BIT_ENC_MICROSTEP_1 = 1;
  static const uint8_t BIT_ENC_EN_2 = 2;
  static const uint8_t BIT_ENC_MICROSTEP_2 = 3;
  static const uint8_t BIT_ENC_EN_3 = 4;
  static const uint8_t BIT_ENC_MICROSTEP_3 = 5;
  static const uint8_t BIT_ENC_EN_4 = 6;
  static const uint8_t BIT_ENC_MICROSTEP_4 = 7;
  
  static const uint8_t REG_ENC_1_CFG = 0x05;
  static const uint8_t REG_ENC_1_COUNT = 0x06;
  static const uint8_t REG_ENC_2_CFG = 0x07;
  static const uint8_t REG_ENC_2_COUNT = 0x08;
  static const uint8_t REG_ENC_3_CFG = 0x09;
  static const uint8_t REG_ENC_3_COUNT = 0x0A;
  static const uint8_t REG_ENC_4_CFG = 0x0B;
  static const uint8_t REG_ENC_4_COUNT = 0x0C;
  
  //Cap touch
  static const uint8_t REG_CAPTOUCH_EN = 0x0D;
  static const uint8_t REG_CAPTOUCH_CFG = 0x0E;
  static const uint8_t REG_CAPTOUCH_0 = 0x0F;  //First of 8 bytes from 15-22
  
  //Switch counters
  static const uint8_t REG_SWITCH_EN_P0 = 0x17;
  static const uint8_t REG_SWITCH_EN_P1 = 0x18;
  static const uint8_t REG_SWITCH_P00 = 0x19;  //First of 8 bytes from 25-40
  static const uint8_t REG_SWITCH_P10 = 0x21;  //First of 8 bytes from 33-49
  
  static const uint8_t REG_USER_FLASH = 0xD0;
  static const uint8_t REG_FLASH_PAGE = 0xF0;
  static const uint8_t REG_DEBUG = 0xF8;
  
  static const uint8_t REG_P0 = 0x40;       //protect_bits 2 # Bit addressing
  static const uint8_t REG_SP = 0x41;       //Read only
  static const uint8_t REG_DPL = 0x42;      //Read only
  static const uint8_t REG_DPH = 0x43;      //Read only
  static const uint8_t REG_RCTRIM0 = 0x44;  //Read only
  static const uint8_t REG_RCTRIM1 = 0x45;  //Read only
  static const uint8_t REG_RWK = 0x46;
  static const uint8_t REG_PCON = 0x47;     //Read only
  static const uint8_t REG_TCON = 0x48;
  static const uint8_t REG_TMOD = 0x49;
  static const uint8_t REG_TL0 = 0x4a;
  static const uint8_t REG_TL1 = 0x4b;
  static const uint8_t REG_TH0 = 0x4c;
  static const uint8_t REG_TH1 = 0x4d;
  static const uint8_t REG_CKCON = 0x4e;
  static const uint8_t REG_WKCON = 0x4f;    //Read only
  static const uint8_t REG_P1 = 0x50;       //protect_bits 3 6 # Bit addressing
  static const uint8_t REG_SFRS = 0x51;     //TA protected # Read only
  static const uint8_t REG_CAPCON0 = 0x52;
  static const uint8_t REG_CAPCON1 = 0x53;
  static const uint8_t REG_CAPCON2 = 0x54;
  static const uint8_t REG_CKDIV = 0x55;
  static const uint8_t REG_CKSWT = 0x56;    //TA protected # Read only
  static const uint8_t REG_CKEN = 0x57;     //TA protected # Read only
  static const uint8_t REG_SCON = 0x58;
  static const uint8_t REG_SBUF = 0x59;
  static const uint8_t REG_SBUF_1 = 0x5a;
  static const uint8_t REG_EIE = 0x5b;      //Read only
  static const uint8_t REG_EIE1 = 0x5c;     //Read only
  static const uint8_t REG_CHPCON = 0x5f;   //TA protected # Read only
  static const uint8_t REG_P2 = 0x60;       //Bit addressing
  static const uint8_t REG_AUXR1 = 0x62;
  static const uint8_t REG_BODCON0 = 0x63;  //TA protected
  static const uint8_t REG_IAPTRG = 0x64;   //TA protected # Read only
  static const uint8_t REG_IAPUEN = 0x65;   //TA protected # Read only
  static const uint8_t REG_IAPAL = 0x66;    //Read only
  static const uint8_t REG_IAPAH = 0x67;    //Read only
  static const uint8_t REG_IE = 0x68;       //Read only
  static const uint8_t REG_SADDR = 0x69;
  static const uint8_t REG_WDCON = 0x6a;    //TA protected
  static const uint8_t REG_BODCON1 = 0x6b;  //TA protected
  static const uint8_t REG_P3M1 = 0x6c;
  static const uint8_t REG_P3S = 0xc0;      //Page 1 # Reassigned from 0x6c to avoid collision
  static const uint8_t REG_P3M2 = 0x6d;
  static const uint8_t REG_P3SR = 0xc1;     //Page 1 # Reassigned from 0x6d to avoid collision
  static const uint8_t REG_IAPFD = 0x6e;    //Read only
  static const uint8_t REG_IAPCN = 0x6f;    //Read only
  static const uint8_t REG_P3 = 0x70;       //Bit addressing
  static const uint8_t REG_P0M1 = 0x71;     //protect_bits  2
  static const uint8_t REG_P0S = 0xc2;      //Page 1 # Reassigned from 0x71 to avoid collision
  static const uint8_t REG_P0M2 = 0x72;     //protect_bits  2
  static const uint8_t REG_P0SR = 0xc3;     //Page 1 # Reassigned from 0x72 to avoid collision
  static const uint8_t REG_P1M1 = 0x73;     //protect_bits  3 6
  static const uint8_t REG_P1S = 0xc4;      //Page 1 # Reassigned from 0x73 to avoid collision
  static const uint8_t REG_P1M2 = 0x74;     //protect_bits  3 6
  static const uint8_t REG_P1SR = 0xc5;     //Page 1 # Reassigned from 0x74 to avoid collision
  static const uint8_t REG_P2S = 0x75;
  static const uint8_t REG_IPH = 0x77;      //Read only
  static const uint8_t REG_PWMINTC = 0xc6;  //Page 1 # Read only # Reassigned from 0x77 to avoid collision
  static const uint8_t REG_IP = 0x78;       //Read only
  static const uint8_t REG_SADEN = 0x79;
  static const uint8_t REG_SADEN_1 = 0x7a;
  static const uint8_t REG_SADDR_1 = 0x7b;
  static const uint8_t REG_I2DAT = 0x7c;    //Read only
  static const uint8_t REG_I2STAT = 0x7d;   //Read only
  static const uint8_t REG_I2CLK = 0x7e;    //Read only
  static const uint8_t REG_I2TOC = 0x7f;    //Read only
  static const uint8_t REG_I2CON = 0x80;    //Read only
  static const uint8_t REG_I2ADDR = 0x81;   //Read only
  static const uint8_t REG_ADCRL = 0x82;
  static const uint8_t REG_ADCRH = 0x83;
  static const uint8_t REG_T3CON = 0x84;
  static const uint8_t REG_PWM4H = 0xc7;    //Page 1 # Reassigned from 0x84 to avoid collision
  static const uint8_t REG_RL3 = 0x85;
  static const uint8_t REG_PWM5H = 0xc8;    //Page 1 # Reassigned from 0x85 to avoid collision
  static const uint8_t REG_RH3 = 0x86;
  static const uint8_t REG_PIOCON1 = 0xc9;  //Page 1 # Reassigned from 0x86 to avoid collision
  static const uint8_t REG_TA = 0x87;       //Read only
  static const uint8_t REG_T2CON = 0x88;
  static const uint8_t REG_T2MOD = 0x89;
  static const uint8_t REG_RCMP2L = 0x8a;
  static const uint8_t REG_RCMP2H = 0x8b;
  static const uint8_t REG_TL2 = 0x8c;
  static const uint8_t REG_PWM4L = 0xca;    //Page 1 # Reassigned from 0x8c to avoid collision
  static const uint8_t REG_TH2 = 0x8d;
  static const uint8_t REG_PWM5L = 0xcb;    //Page 1 # Reassigned from 0x8d to avoid collision
  static const uint8_t REG_ADCMPL = 0x8e;
  static const uint8_t REG_ADCMPH = 0x8f;
  static const uint8_t REG_PSW = 0x90;      //Read only
  static const uint8_t REG_PWMPH = 0x91;
  static const uint8_t REG_PWM0H = 0x92;
  static const uint8_t REG_PWM1H = 0x93;
  static const uint8_t REG_PWM2H = 0x94;
  static const uint8_t REG_PWM3H = 0x95;
  static const uint8_t REG_PNP = 0x96;
  static const uint8_t REG_FBD = 0x97;
  static const uint8_t REG_PWMCON0 = 0x98;
  static const uint8_t REG_PWMPL = 0x99;
  static const uint8_t REG_PWM0L = 0x9a;
  static const uint8_t REG_PWM1L = 0x9b;
  static const uint8_t REG_PWM2L = 0x9c;
  static const uint8_t REG_PWM3L = 0x9d;
  static const uint8_t REG_PIOCON0 = 0x9e;
  static const uint8_t REG_PWMCON1 = 0x9f;
  static const uint8_t REG_ACC = 0xa0;      //Read only
  static const uint8_t REG_ADCCON1 = 0xa1;
  static const uint8_t REG_ADCCON2 = 0xa2;
  static const uint8_t REG_ADCDLY = 0xa3;
  static const uint8_t REG_C0L = 0xa4;
  static const uint8_t REG_C0H = 0xa5;
  static const uint8_t REG_C1L = 0xa6;
  static const uint8_t REG_C1H = 0xa7;
  static const uint8_t REG_ADCCON0 = 0xa8;
  static const uint8_t REG_PICON = 0xa9;    //Read only
  static const uint8_t REG_PINEN = 0xaa;    //Read only
  static const uint8_t REG_PIPEN = 0xab;    //Read only
  static const uint8_t REG_PIF = 0xac;      //Read only
  static const uint8_t REG_C2L = 0xad;
  static const uint8_t REG_C2H = 0xae;
  static const uint8_t REG_EIP = 0xaf;      //Read only
  static const uint8_t REG_B = 0xb0;        //Read only
  static const uint8_t REG_CAPCON3 = 0xb1;
  static const uint8_t REG_CAPCON4 = 0xb2;
  static const uint8_t REG_SPCR = 0xb3;
  static const uint8_t REG_SPCR2 = 0xcc;    //Page 1 # Reassigned from 0xb3 to avoid collision
  static const uint8_t REG_SPSR = 0xb4;
  static const uint8_t REG_SPDR = 0xb5;
  static const uint8_t REG_AINDIDS = 0xb6;
  static const uint8_t REG_EIPH = 0xb7;     //Read only
  static const uint8_t REG_SCON_1 = 0xb8;
  static const uint8_t REG_PDTEN = 0xb9;    //TA protected
  static const uint8_t REG_PDTCNT = 0xba;   //TA protected
  static const uint8_t REG_PMEN = 0xbb;
  static const uint8_t REG_PMD = 0xbc;
  static const uint8_t REG_EIP1 = 0xbe;     //Read only
  static const uint8_t REG_EIPH1 = 0xbf;    //Read only
  
  
  static const uint8_t REG_INT = 0xf9;
  static const uint8_t MASK_INT_TRIG = 0x1;
  static const uint8_t MASK_INT_OUT = 0x2;
  static const uint8_t BIT_INT_TRIGD = 0;
  static const uint8_t BIT_INT_OUT_EN = 1;
  static const uint8_t BIT_INT_PIN_SWAP = 2;  //0 = P1.3, 1 = P0.0
  
  static const uint8_t REG_INT_MASK_P0 = 0x00;
  static const uint8_t REG_INT_MASK_P1 = 0x01;
  static const uint8_t REG_INT_MASK_P3 = 0x03;
  
 
  static const uint8_t REG_ADDR = 0xfd;
  
  static const uint8_t REG_CTRL = 0xfe;     //0 = Sleep, 1 = Reset, 2 = Read Flash, 3 = Write Flash, 4 = Addr Unlock
  static const uint8_t MASK_CTRL_SLEEP = 0x1;
  static const uint8_t MASK_CTRL_RESET = 0x2;
  static const uint8_t MASK_CTRL_FREAD = 0x4;
  static const uint8_t MASK_CTRL_FWRITE = 0x8;
  static const uint8_t MASK_CTRL_ADDRWR = 0x10;
  
  //Special mode registers, use a bit-addressing scheme to avoid
  //writing the *whole* port and smashing the i2c pins
  static const uint8_t NUM_BIT_ADDRESSED_REGISTERS = 4;
  static const uint8_t BIT_ADDRESSED_REGS[NUM_BIT_ADDRESSED_REGISTERS]; //[REG_P0, REG_P1, REG_P2, REG_P3]

  static const uint8_t ENC_CFG[4];   //[REG_ENC_1_CFG, REG_ENC_2_CFG, REG_ENC_3_CFG, REG_ENC_4_CFG]
  static const uint8_t ENC_COUNT[4]; //[REG_ENC_1_COUNT, REG_ENC_2_COUNT, REG_ENC_3_COUNT, REG_ENC_4_COUNT]
 
   //These values encode our desired pin function: IO, ADC, PWM
  //alongwide the GPIO MODE for that port and pin (section 8.1)
  //1st and 2nd bits encode the gpio state
  //3rd and 4th bits encode the IO mode (i.e. IO, PWM, ADC)
  //the 5th bit additionally encodes the default output state
  static const uint8_t PIN_MODE_IO = 0b00000;   //General IO mode, IE: not ADC or PWM
  static const uint8_t PIN_MODE_QB = 0b00000;   //Output, Quasi-Bidirectional mode
  static const uint8_t PIN_MODE_PP = 0b00001;   //Output, Push-Pull mode
  static const uint8_t PIN_MODE_IN = 0b00010;   //Input-only (high-impedance)
  static const uint8_t PIN_MODE_PU = 0b10000;   //Input (with pull-up)
  static const uint8_t PIN_MODE_OD = 0b00011;   //Output, Open-Drain mode
  static const uint8_t PIN_MODE_PWM = 0b00101;  //PWM, Output, Push-Pull mode
  static const uint8_t PIN_MODE_ADC = 0b01010;  //ADC, Input-only (high-impedance)

  static const char* MODE_NAMES[3];  //['IO', 'PWM', 'ADC']
  static const char* GPIO_NAMES[4];  //['QB', 'PP', 'IN', 'OD']
  static const char* STATE_NAMES[2]; //['LOW', 'HIGH']
public:
  static const uint8_t PIN_IN = PIN_MODE_IN;          //0b00010
  static const uint8_t PIN_IN_PULL_UP = PIN_MODE_PU;  //0b10000
  static const uint8_t PIN_IN_PU = PIN_MODE_PU;       //0b10000
  static const uint8_t PIN_OUT = PIN_MODE_PP;         //0b00001
  static const uint8_t PIN_PWM = PIN_MODE_PWM;        //0b00101
  static const uint8_t PIN_ADC = PIN_MODE_ADC;        //0b01010

  static const uint8_t NUM_PINS = 14;
  

  //--------------------------------------------------
  // Subclasses
  //--------------------------------------------------
private:
  class Pin
  {
  public:
    //--------------------------------------------------
    // Enums
    //--------------------------------------------------
    enum IOType
    {
      TYPE_IO =         0b00,
      TYPE_PWM =        0b01,
      TYPE_ADC =        0b10,
      TYPE_ADC_OR_PWM = 0b11
    };
  
    //--------------------------------------------------
    // Constants
    //--------------------------------------------------
  private:
    //The PxM1 and PxM2 registers encode GPIO MODE
    //0 0 = Quasi-bidirectional
    //0 1 = Push-pull
    //1 0 = Input-only (high-impedance)
    //1 1 = Open-drain
    static const uint8_t PxM1[4];     //[REG_P0M1, REG_P1M1, -1, REG_P3M1]
    static const uint8_t PxM2[4];     //[REG_P0M2, REG_P1M2, -1, REG_P3M2]

    //The Px input register
    static const uint8_t Px[4];       //[REG_P0, REG_P1, -1, REG_P3]

    //The PxS Schmitt trigger register
    static const uint8_t PxS[4];      //[REG_P0S, REG_P1S, -1, REG_P3S]
    static const uint8_t MASK_P[4];   //[REG_INT_MASK_P0, REG_INT_MASK_P1, -1, REG_INT_MASK_P3]

    static const uint8_t PWML[6];     //[REG_PWM0L, REG_PWM1L, REG_PWM2L, REG_PWM3L, REG_PWM4L, REG_PWM5L]
    static const uint8_t PWMH[6];     //[REG_PWM0H, REG_PWM1H, REG_PWM2H, REG_PWM3H, REG_PWM4H, REG_PWM5H]


    //--------------------------------------------------
    // Constructors/Destructor
    //--------------------------------------------------
  private:
    Pin(uint8_t port, uint8_t pin);                                                               //Constructor for IO pin
    Pin(uint8_t port, uint8_t pin, uint8_t pwm_channel, uint8_t reg_iopwm);                       //Constructor for PWM pin
    Pin(uint8_t port, uint8_t pin, uint8_t adc_channel);                                          //Constructor for ADC pin
    Pin(uint8_t port, uint8_t pin, uint8_t adc_channel, uint8_t pwm_channel, uint8_t reg_iopwm);  //Constructor for ADC or PWM pin

    
    //--------------------------------------------------
    // Methods
    //--------------------------------------------------
  public:    
    static Pin Io(uint8_t port, uint8_t pin);                                                                     //Nicer function for creating an IO pin
    static Pin Pwm(uint8_t port, uint8_t pin, uint8_t channel, uint8_t reg_iopwm);                                //Nicer function for creating a PWM pin
    static Pin Adc(uint8_t port, uint8_t pin, uint8_t channel);                                                   //Nicer function for creating an ADC pin    
    static Pin AdcOrPwm(uint8_t port, uint8_t pin, uint8_t adc_channel, uint8_t pwm_channel, uint8_t reg_iopwm);  //Nicer function for creating an ADC or PWM pin

    //--------------------------------------------------

    IOType getType(void);    
    uint8_t getMode(void);
    void setMode(uint8_t mode);
    
    bool modeSupported(uint8_t mode);
    

    //--------------------------------------------------
    // Variables
    //--------------------------------------------------
  private:
    const IOType _type;    
    uint8_t _mode;
  public:
    const uint8_t _port;
    const uint8_t _pin;
    const uint8_t _adcChannel;
    const uint8_t _pwmChannel;
    
    const uint8_t _reg_m1;
    const uint8_t _reg_m2;
    const uint8_t _reg_p;
    const uint8_t _reg_ps;
    const uint8_t _reg_int_mask_p;

    const uint8_t _regIoPwm;
    const uint8_t _reg_pwml;
    const uint8_t _reg_pwmh;
  };


  //--------------------------------------------------
  // Constructors/Destructor
  //--------------------------------------------------
public:
  IOExpander(TwoWire& wire, uint8_t address = DEFAULT_I2C_ADDRESS, uint32_t timeout = 1,
             int8_t interruptPin = -1, bool debug = false);


  //--------------------------------------------------
  // Methods
  //--------------------------------------------------
public:
  bool initialise(bool skipChipIdCheck = false);

  //--------------------------------------------------

  uint16_t getChipId(void);
  
  void setAddr(uint8_t i2cAddr);

  float getAdcVref(void);
  void setAdcVref(float vref);
  
  //--------------------------------------------------
  
  void enableInterruptOut(bool pinSwap = false);
  void disableInterruptOut(void);

  uint8_t getInterruptFlag(void);
  void clearInterruptFlag(void);

  bool setPinInterrupt(uint8_t pin, bool enabled);
  void setInterruptCallback(void (*callback)());

  //--------------------------------------------------

  void pwmLoad(bool waitForLoad = true);
  bool pwmLoading(void);

  void pwmClear(bool waitForClear = true);
  bool pwmClearing(void);

  bool setPwmControl(uint8_t divider);
  void setPwmPeriod(uint16_t value, bool load = true);

  //--------------------------------------------------

  uint8_t getMode(uint8_t pin);
  void setMode(uint8_t pin, uint8_t mode, bool schmittTrigger = false, bool invert = false);

  int16_t input(uint8_t pin, uint32_t adcTimeout = 1);
  float inputAsVoltage(uint8_t pin, uint32_t adcTimeout = 1);
  
  void output(uint8_t pin, uint16_t value, bool load = true);

  //--------------------------------------------------
  
  void setupRotaryEncoder(uint8_t channel, uint8_t pinA, uint8_t pinB, uint8_t pinC = 0, bool countMicrosteps = false);
  uint16_t readRotaryEncoder(uint8_t channel);

  //--------------------------------------------------
private:
  uint8_t i2cRead8(uint8_t reg);
  void i2cWrite8(uint8_t reg, uint8_t value);

  uint8_t getBit(uint8_t reg, uint8_t bit);
  void setBits(uint8_t reg, uint8_t bits);
  void setBit(uint8_t reg, uint8_t bit);
  void clrBits(uint8_t reg, uint8_t bits);
  void clrBit(uint8_t reg, uint8_t bit); 
  void changeBit(uint8_t reg, uint8_t bit, bool state);

  void waitForFlash(void);


  //--------------------------------------------------
  // Variables
  //--------------------------------------------------
private:
  uint8_t _i2cAddr;
  TwoWire& _wire;
  bool _debug;
  float _vref;
  uint32_t _timeout;
  uint8_t _interruptPin;
  uint8_t _encoderOffset[4];
  uint8_t _encoderLast[4];

  Pin _pins[NUM_PINS];
};
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
