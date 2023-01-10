
/*
 * Mics6814 Breakout example
 * 
 * Ported from https://github.com/pimoroni/pimoroni-pico/tree/main/examples/breakout_mics6814
 * by Tim Lunn (@darkxst) 
 */

/***** Library Includes *****/
#include "GasBreakout.h"

/***** Global Constants *****/
static const unsigned int DELAY_MS = 15 * 1000;

/***** Global Variables *****/
GasBreakout gas(Wire, 0x19);

void setup() {

  Serial.begin(9600);
  delay(1000);
  
  Wire.begin();
  
  if(!gas.initialise()){
      Serial.println("MICS6814 - Not Initialised");
      while(true) {}
  }
  Serial.println("MICS6814 - Initialised");

}

void loop() {

  GasBreakout::Reading reading;
  reading = gas.readAll();

  Serial.println("Reading Gas");
  Serial.print("Red: ");
  Serial.println(reading.reducing);
  Serial.print("NH3: ");
  Serial.println(reading.nh3);
  Serial.print("Ox: ");
  Serial.println(reading.oxidising);
  Serial.println("");

  delay(DELAY_MS);
}
