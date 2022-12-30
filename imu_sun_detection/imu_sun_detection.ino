/*
 * imu_sun_detection.ino
 *
 * Uses known values of latitude,
 * longitude, time of year, and time of
 * day to determine the azimuth and elevation
 * of the Sun in the sky. With these values,
 * use the IMU to detect if it is pointed at
 * the Sun, and if not how to point at it.
 *
 * This program uses _active calibration_ where we
 * constantly check the IMU's calibration registers
 * before capturing data points. If the system is
 * not calibrated, then...
 *
 * Author: Ethan Garnier
 * Date Modified: December 30, 2022
 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimeLib.h>
#include "sun_pos.h"

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Object representing IMU through Adafruit's library
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Globals representing IMU calibration register values
uint8_t systemCal, gyroCal, accelCal, magCal;

// Sun angles
float azimuth = 0.0f;
float elevation = 0.0f;

// Variable that keeps track of current time
time_t currentTime;

/*
 Determine the Sun's azimuth and elevation
 angles based on the current date, time,
 latitude and longitude
*/
void findSunAngles()
{
  // todo: TIME ZONE???????
  calcSunPos(&elevation, &azimuth, -66.21098, 45.36305, currentTime);
}

/*
 * Calibrate each sensor of the BNO055
 */
void calibrateDevice()
{
  // Get the four calibration values (0..3)
  // Any sensor data reporting 0 should be ignored
  // 3 means 'fully calibrated"
  systemCal = gyroCal = accelCal = magCal = 0;

  Serial.print("\n-----------\n");
  Serial.print("Calibrating BNO055 Device:\n");

  Serial.print("\tTo calibrate the Gyroscope, place the device down and let rest still...\n");
  while (gyroCal != 3)
  {
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
  }
  Serial.print("\tDone calibrating Gyroscope\n");

  Serial.print("\tTo calibrate the Accelerometer, rotate the device around an axis in 45 degree increments...\n");
  while (accelCal != 3)
  {
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
  }
  Serial.print("\tDone calibrating Accelerometer\n");

  Serial.print("\tTo calibrate the Magnetometer, move the device through the air in a figure-8 pattern...\n");
  while (magCal != 3)
  {
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
  }
  Serial.print("\tDone calibrating Magnetometer\n");

  while (systemCal != 3)
  {
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
  }

  Serial.print("Calibration complete!\n");
  Serial.print("-----------\n\n");
}

void setup()
{
  Serial.begin(9600);
  Serial.print("BNO055 Sun Detection\n\n");

  // TODO:
  // Manually set date and time, in the future
  // maybe use a real time clock? 
  // TIME MUST BE IN UTC
  setTime(19, 30, 0, 30, 12, 2022);

  // Get current date and time
  currentTime = now();

  // Initialise the sensor
  /*if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }*/

  delay(1000);

  //calibrateDevice();

  //bno.setExtCrystalUse(true);
}

void loop()
{
  findSunAngles();

  Serial.print("Sun's elevation angle: ");
  Serial.print(elevation, 3);
  Serial.println();
  Serial.print("Sun's azimuth angle: ");
  Serial.print(azimuth, 3);
  Serial.println();

  delay(10000);
}
