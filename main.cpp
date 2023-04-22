// SenESP Engine Sensors

#include <Adafruit_BMP280.h>
#include <Wire.h>

#include "sensesp_onewire/onewire_temperature.h"

#include <Arduino.h>

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/sensors/digital_input.h"


using namespace sensesp;


class TemperatureInterpreter : public CurveInterpolator {
 public:
  TemperatureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
    add_sample(CurveInterpolator::Sample(20, 393.15));
    add_sample(CurveInterpolator::Sample(30, 383.15));
    add_sample(CurveInterpolator::Sample(40, 373.15));
    add_sample(CurveInterpolator::Sample(55, 363.15));
    add_sample(CurveInterpolator::Sample(70, 353.15));
    add_sample(CurveInterpolator::Sample(100, 343.15));
    add_sample(CurveInterpolator::Sample(140, 333.15));
    add_sample(CurveInterpolator::Sample(200, 323.15));
    add_sample(CurveInterpolator::Sample(300, 317.15));
    add_sample(CurveInterpolator::Sample(400, 313.15)); 
  }
};

class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to LPH
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, LPH));
    add_sample(CurveInterpolator::Sample(500, 0.4));
    add_sample(CurveInterpolator::Sample(1000, 0.7));
    add_sample(CurveInterpolator::Sample(1500, 1.1));
    add_sample(CurveInterpolator::Sample(1800, 1.5));
    add_sample(CurveInterpolator::Sample(2000, 1.9));
    add_sample(CurveInterpolator::Sample(2200, 2.4));
    add_sample(CurveInterpolator::Sample(2400, 2.85));
    add_sample(CurveInterpolator::Sample(2600, 3.5));
    add_sample(CurveInterpolator::Sample(2800, 4.45));
    add_sample(CurveInterpolator::Sample(3000, 5.5));
    add_sample(CurveInterpolator::Sample(3200, 6.6));
    add_sample(CurveInterpolator::Sample(3400, 7.2));
    add_sample(CurveInterpolator::Sample(3800, 7.4));  
  }
};

reactesp::ReactESP app;

  Adafruit_BMP280 bmp280;

  float read_temp_callback() { return (bmp280.readTemperature() + 273.15);}
  float read_pressure_callback() { return (bmp280.readPressure());}

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("SensESP")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_uptime_sensor()
                    ->get_app();

/// 1-Wire Temp Sensors - Exhaust Temp Sensors ///

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(17);

  auto* exhaust_temp =
      new OneWireTemperature(dts, 1000, "/Exhaust Temperature/oneWire");
 
  exhaust_temp->connect_to(new Linear(1.0, 0.0, "/Exhaust Temperature/linear"))
      ->connect_to(new SKOutputFloat("propulsion.main.exhaustTemperature","/Exhaust Temperature/sk_path"));


 //RPM Application/////

  const char* config_path_calibrate = "/Engine RPM/calibrate";
  const char* config_path_skpath = "/Engine RPM/sk_path";
  const float multiplier = 1.0;

  auto* sensor = new DigitalInputCounter(16, INPUT_PULLUP, RISING, 500);

  sensor->connect_to(new Frequency(multiplier, config_path_calibrate))  
  // connect the output of sensor to the input of Frequency()
          >connect_to(new MovingAverage(2, 1.0,"/Engine RPM/movingAVG"))
         ->connect_to(new SKOutputFloat("propulsion.main.revolutions", config_path_skpath));  
          // connect the output of Frequency() to a Signal K Output as a number

  sensor->connect_to(new Frequency(6))
  // times by 6 to go from Hz to RPM
          ->connect_to(new MovingAverage(4, 1.0,"/Engine Fuel/movingAVG"))
          ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
          ->connect_to(new SKOutputFloat("propulsion.engine.fuelconsumption", "/Engine Fuel/sk_path"));                                       

/// BMP280 SENSOR CODE - Engine Room Temp Sensor ////  

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bmp280.begin(0x76);

  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* engine_room_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto* engine_room_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);

  // Send the temperature to the Signal K server as a Float
  engine_room_temp->connect_to(new SKOutputFloat("propulsion.engineRoom.temperature"));
  engine_room_pressure->connect_to(new SKOutputFloat("propulsion.engineRoom.pressure"));

//// Engine Temp Config ////

const float Vin = 3.3;
const float R1 = 120.0;
auto* analog_input = new AnalogInput(36, 2000);

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new TemperatureInterpreter("/Engine Temp/curve"))
      ->connect_to(new Linear(1.0, 0.9, "/Engine Temp/calibrate"))
      ->connect_to(new MovingAverage(4, 1.0,"/Engine Temp/movingAVG"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature", "/Engine Temp/sk_path"));
 
 //// Bilge Monitor /////

auto* bilge = new DigitalInputState(25, INPUT_PULLUP, 5000);

auto int_to_string_function = [](int input) ->String {
     if (input == 1) {
       return "Water present!";
     } 
     else { // input == 0
       return "bilge clear";
     }
};

auto int_to_string_transform = new LambdaTransform<int, String>(int_to_string_function);

bilge->connect_to(int_to_string_transform)
      ->connect_to(new SKOutputString("propulsion.engine.bilge"));

bilge->connect_to(new SKOutputString("propulsion.engine.bilge.raw"));


  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
