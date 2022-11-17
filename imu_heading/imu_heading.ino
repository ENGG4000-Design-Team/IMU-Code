/*
* Calibrate and reading the heading
* data from a BNO055 IMU
*
* Author: Ethan Garnier
* Date: November 17, 2022
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*
* Calibrate each sensor of the BNO055
*/
void calibrateDevice() 
{
  // Get the four calibration values (0..3)
  // Any sensor data reporting 0 should be ignored
  // 3 means 'fully calibrated"
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;

  Serial.print("\n-----------\n");
  Serial.print("Calibrating BNO055 Device:\n");

  Serial.print("\tTo calibrate the Gyroscope, place the device down and let rest still...\n");
  while (gyro != 3){bno.getCalibration(&system, &gyro, &accel, &mag);}
  Serial.print("\tDone calibrating Gyroscope\n");

  Serial.print("\tTo calibrate the Accelerometer, rotate the device around an axis in 45 degree increments...\n");    
  while (accel != 3){bno.getCalibration(&system, &gyro, &accel, &mag);}
  Serial.print("\tDone calibrating Accelerometer\n");

  Serial.print("\tTo calibrate the Magnetometer, move the device through the air in a figure-8 pattern...\n");
  while (mag != 3){bno.getCalibration(&system, &gyro, &accel, &mag);}
  Serial.print("\tDone calibrating Magnetometer\n");

  while (system != 3){bno.getCalibration(&system, &gyro, &accel, &mag);}

  Serial.print("Calibration complete!\n");
  Serial.print("-----------\n\n");
}

void setup() 
{
  Serial.begin(9600);
  Serial.print("BNO055 Heading Reader\n\n"); 
  
  // Initialise the sensor
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  calibrateDevice();

  bno.setExtCrystalUse(true);  
}

void loop() 
{
  // Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  // Display the floating point data
  Serial.print("Heading: ");
  Serial.print(event.orientation.x, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4);

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}