#include <Arduino_APDS9960.h>
#include "pid_controller.h"

namespace {

constexpr auto led_pin = PIN_A0; // LED_BUILTIN;
pid_controller<float> ctrl;
float kp = 0.001;
float ki = 0.001;
float kd = 0.01;
float target = 128;

}

void setup() 
{
  ctrl.set_gains(kp, ki, kd);
  ctrl.set_limits(0, 255);
  ctrl.set_target(target);

  Serial.begin(115200);
  while (!Serial);

  if (!APDS.begin()) {
    Serial.println("error initializing APDS9960 sensor");
    while (true);
  }
  pinMode(led_pin, OUTPUT);
}

void loop() 
{
  static unsigned prev_ts;
  static int light_r;
  static int light_g;
  static int light_b;
  static int light_c;

  if (APDS.colorAvailable() && (millis() - prev_ts) >= 10) {
    APDS.readColor(light_r, light_g, light_b, light_c);
    auto ts = millis();
    auto output = ctrl.compute(light_c, ts - prev_ts);
    prev_ts = ts;
    analogWrite(led_pin, output);
    Serial.print("target:");
    Serial.print(target);
    Serial.print(",");
    Serial.print("light:");
    Serial.print(light_c);
    Serial.print(",");
    Serial.print("output:");
    Serial.println(output);
  } 

  if (Serial.available() > 1) {
    char c = Serial.read();
    float val = Serial.parseFloat();
    switch (c)
    {
    case 'p': kp = val; break;
    case 'i': ki = val; break;
    case 'd': kd = val; break;
    case 't': target = val; break;
    }
    ctrl.reset_state();
    ctrl.set_gains(kp, ki, kd);
    ctrl.set_target(target);
  }
}
