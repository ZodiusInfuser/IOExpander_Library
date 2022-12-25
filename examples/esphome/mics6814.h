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
          ESP_LOGD("custom","MICS6814 - Not Initialised");
          while(true) {}
      }
      ESP_LOGD("custom", "MICS6814 - Initialised");
}

  void update() override {

    GasBreakout::Reading reading;
    reading = gas.readAll();

    red_sensor->publish_state(reading.reducing);
    nh3_sensor->publish_state(reading.nh3);
    ox_sensor->publish_state(reading.oxidising);
  
  }

};

class HeaterSwitch : public Component, public Switch {
  public:

    float get_setup_priority() const override { return esphome::setup_priority::LATE; }

    void setup() override {
      publish_state(true);
    }

    void write_state(bool state) override {
      gas.setHeater(state);
      publish_state(state);
    }
};

class LedChannel : public Component, public FloatOutput {
  public:
    float get_setup_priority() const override { return esphome::setup_priority::LATE; }
    void (GasBreakout::*ch_handler)(uint8_t v);

    LedChannel(char chan) {
      switch(chan)
      {
        case 'R': ch_handler=&GasBreakout::setR; break;
        case 'G': ch_handler=&GasBreakout::setG; break;
        case 'B': ch_handler=&GasBreakout::setB; break;
        default:
          ch_handler=NULL;
          ESP_LOGD("custom", "no callback");
      }
    }

    void write_state(float state) override {
      uint8_t period = state * 255;
      if (ch_handler != NULL)
        (&gas->*ch_handler)(period);
    }
};
