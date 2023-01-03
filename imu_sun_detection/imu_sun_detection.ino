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
 * Date Modified: January 2, 2022
 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimeLib.h>
#include "sun_pos.h"
#include "time_impl.h"

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Object representing IMU through Adafruit's library
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Globals representing IMU calibration register values
uint8_t systemCal, gyroCal, accelCal, magCal;

// Sun angles
float azimuth = 0.0f;
float elevation = 0.0f;

// Latitude and longitude of device
float latitude = 45.94505;
float longitude = -66.64798;

// Correction angles
float corrAzimuth = 0.0f;
float corrElevation = 0.0f;

// AST time zone
int utc_offset = -4;

/*
 Determine the difference between the Sun's
 azimuth and elevation angles and the data
 being reported by the IMU.
*/
void determineCorrectionAngles()
{
  // Determine the Sun's azimuth and elevation
  // angles based on the current date, time,
  // latitude and longitude
  // todo: currently latitude and longitude are hardcoded

  // Get current date and time before calculating sun's position
  time_t utc = now();
  calcSunPos(&elevation, &azimuth, longitude, latitude, utc);

  // Grab heading and elevation values from IMU
  // IFF the calibration registers are reading 3
  while (systemCal != 3 ||
         gyroCal != 3 ||
         accelCal != 3 ||
         magCal != 3)
  {
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
  }

  // When we know the system is fully calibrated then continue
  sensors_event_t event;
  bno.getEvent(&event);

  // TODO: I DO NOT KNOW IF THIS WORKS
  // Calculate the required correction angles as the difference
  corrAzimuth = azimuth - event.orientation.x;
  corrElevation = elevation - event.orientation.y;
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

  // Set time to when the project was compiled
  // converted to UTC
  setTime(toUTC(compileTime()));

  // Initialise the sensor
  if (!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  calibrateDevice();

  bno.setExtCrystalUse(true);
}

void loop()
{
  determineCorrectionAngles();

  if (corrElevation < 0.01 && corrAzimuth < 0.01)
  {
    Serial.println("YOU ARE POINTING AT THE SUN!");
  }

  Serial.println();
  Serial.print("Elevation correction angle (deg): ");
  Serial.print(corrElevation, 3);
  Serial.println();
  Serial.print("Azimuth correction angle (deg): ");
  Serial.print(corrAzimuth, 3);
  Serial.println();

  delay(10000);
}
