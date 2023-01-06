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
  Grabs the orientation of the setup BNO055
  IMU device in the form Euler angles vector
    .x() is heading / yaw
    .y() is pitch
    .z() is roll

  The returned values are all in degrees.

  The optional useQuat parameter specifies if
  the calculation should be performed on a quaternion
  returned from the IMU, instead of fusion sensor data.
*/
imu::Vector<3> getOrientation(bool useQuat = false)
{
  imu::Vector<3> euler;

  while (!bno.isFullyCalibrated())
  {
    // Only read data from IMU if fully calibrated
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
    Serial.print("Calibration Register Values: ");
    Serial.print(systemCal);
    Serial.print(" ");
    Serial.print(gyroCal);
    Serial.print(" ");
    Serial.print(accelCal);
    Serial.print(" ");
    Serial.print(magCal);
    Serial.println();
  }

  if (useQuat)
  {
    // BROKEN
    imu::Quaternion quat = bno.getQuat();
    quat.normalize();

    // TODO: Evaluate addressing singularities
    // and swapping axis for application
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
    // https://forums.adafruit.com/viewtopic.php?f=19&t=91723&p=462337#p462337

    euler = quat.toEuler();
    euler.x() = -180 / PI * euler.x();
    euler.y() = -180 / PI * euler.y();
    euler.z() = -180 / PI * euler.z();
  }
  else
  {
    // TODO: Investigate manually calculating Euler angles
    // via this tutorial: https://www.youtube.com/watch?v=yPfQK75dZbU&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9&index=6

    sensors_event_t event;
    bno.getEvent(&event);

    euler.x() = event.orientation.x;
    euler.y() = -event.orientation.z;
    euler.z() = event.orientation.y;
  }

  // DESIRED:
  // heading/yaw, nose-right is positive, z-axis points up
  // pitch, nose-up is positive, x-axis points right
  // roll, rightwing-up is positive, y-axis points forward

  Serial.print("Orientation: ");
  Serial.print(euler.x());
  Serial.print(" ");
  Serial.print(euler.y());
  Serial.print(" ");
  Serial.print(euler.z());
  Serial.println("");

  return euler;
}

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

  // Grab IMU Euler angles
  imu::Vector<3> euler = getOrientation();

  // Printing this for debugging
  Serial.println();
  Serial.println("Measured Azimuth: ");
  Serial.println(euler.x());
  Serial.println("Desired Azimuth: ");
  Serial.println(azimuth);

  Serial.println("Measured Elevation: ");
  Serial.println(euler.y());
  Serial.println("Desired Elevation: ");
  Serial.println(elevation);
  Serial.println();

  // NOTE:
  // We should be using event.orientation.y as the pitch, however
  // for some reason the BNO055 "front" appears to be its left side??
  // We may need to add 90 to event.orientation.x to correct for this?

  // TODO: I DO NOT KNOW IF THIS WORKS
  // Calculate the required correction angles as the difference
  corrAzimuth = azimuth - euler.x();
  corrElevation = elevation - euler.y();
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

  Serial.print("\tTo calibrate the Magnetometer, move the device through the air in a figure-8 pattern...\n");
  while (magCal != 3)
  {
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
  }
  Serial.print("\tDone calibrating Magnetometer\n");

  Serial.print("\tTo calibrate the Accelerometer, rotate the device around an axis in 45 degree increments...\n");
  while (accelCal != 3)
  {
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
  }
  Serial.print("\tDone calibrating Accelerometer\n");

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
  if (!bno.begin(OPERATION_MODE_NDOF))
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(1000);

  // TODO: Do we need this?
  // Remap axis based on chip placement
  // bno.setAxisRemap(0x21);
  // bno.setAxisSign(0x04);

  calibrateDevice();

  bno.setExtCrystalUse(true);
}

void loop()
{
  /*determineCorrectionAngles();

  if (abs(corrElevation) < 0.01 && abs(corrAzimuth) < 0.01)
  {
    Serial.println("YOU ARE POINTING AT THE SUN!");
  }

  Serial.println();
  Serial.print("Elevation correction angle (deg): ");
  Serial.print(corrElevation, 3);
  Serial.println();
  Serial.print("Azimuth correction angle (deg): ");
  Serial.print(corrAzimuth, 3);
  Serial.println();*/

  getOrientation();

  delay(1000);
}
