#include "esphome.h"
#include "GasBreakout.h"

using namespace esphome;

/***** Global Constants *****/
static const unsigned int DELAY_MS = 15 * 1000;

/***** Global Variables *****/
GasBreakout gas(Wire, 0x19);

class mics6814 : public PollingComponent {
 public:
  Sensor *red_sensor = new Sensor();
  Sensor *nh3_sensor = new Sensor();
  Sensor *ox_sensor = new Sensor();

  mics6814() : PollingComponent(DELAY_MS) { }

  float get_setup_priority() const override { return esphome::setup_priority::BUS; }

  void setup() override {

    Serial.begin(9600);
    delay(1000);
    
    Wire.begin();
    
    if(!gas.initialise()){
        ESP_LOGD("custom", "MICS6814 - Not Initialised");
        Serial.println("MICS6814 - Not Initialised");
        while(true) {}
    }
    ESP_LOGD("custom", "MICS6814 - Initialised");
    Serial.println("MICS6814 - Initialised");

  }

  void update() override {

    GasBreakout::Reading reading;
    reading = gas.readAll();

    red_sensor->publish_state(reading.reducing);
    nh3_sensor->publish_state(reading.nh3);
    ox_sensor->publish_state(reading.oxidising);
  
  }
};
