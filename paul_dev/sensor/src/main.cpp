/// Reference: https://learn.adafruit.com/adafruit-9-dof-imu-breakout/software

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Wire.h>

// Timing variables
float loop_start_ms = 0.0;
float loop_end_ms = 0.0;

// 20 Hz
const float CONTROL_CYCLE_TIME_ms = 50.0;

Adafruit_LSM303_Accel_Unified accel;
Adafruit_9DOF dof;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Setting Up...");

  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }

  dof.begin();

}

void loop() {
  loop_start_ms = static_cast<float>(micros()) / 1000.0;

  if (loop_start_ms - loop_end_ms < CONTROL_CYCLE_TIME_ms)
  {
    delay(1); // Wait 1 millisecond in between polls.
    return;
  }

  // Do work here:
  sensors_event_t accel_event;
  sensors_vec_t   orientation;

  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
    Serial.println();
  }

  loop_end_ms = static_cast<float>(micros()) / 1000.0;

  Serial.print("Loop Time ms: ");
  Serial.println(loop_end_ms - loop_start_ms);
}
