/// Reference: https://learn.adafruit.com/adafruit-9-dof-imu-breakout/software

#include <Arduino.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_LSM303.h>
// #include <Adafruit_LSM303_Accel.h>
// #include <Adafruit_L3GD20_U.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_9DOF.h>

// Timing variables
unsigned long startTime = 0;
unsigned long lastEndTime = 0;

// 20 Hz
const uint32_t CONTROL_CYCLE_TIME_ms = 1 / 20 * 1000;

sensors_event_t accel_event;
sensors_vec_t   orientation;

Adafruit_LSM303_Accel_Unified accel;
Adafruit_9DOF dof;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  startTime = millis();

  if (lastEndTime - startTime > CONTROL_CYCLE_TIME_ms)
  {
    delay(1); // Wait 1 millisecond in between polls.
    return;
  }

  // Do work here:
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
  }

  lastEndTime = millis();

  Serial.print("Loop Time: ");
  Serial.println(lastEndTime - startTime);
}
