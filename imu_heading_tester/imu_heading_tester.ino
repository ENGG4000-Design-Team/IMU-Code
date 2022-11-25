/*
* imu_heading_tester.ino
* Calibrates the BNO055 IMU and instructs
* the operator to place the sensor at certain
* headings relative to the North Pole. BNO055
* then takes heading measurements when the operator
* is ready by pressing a button. These heading measrements
* are then output, along with their expected value, to
* the serial monitor. To output these data values to a .txt
* file for further processing, use an application like 
* CoolTerm https://freeware.the-meiers.org/
* 
* This program takes 100 measurements at the following headings:
*   - 0 degrees
*   - 15 degrees
*   - 30 degrees
*   - 45 degrees
*   - 60 degrees
*   - 75 degrees
*   - 90 degrees
*   - 105 degrees
*   - 120 degrees
*   - 135 degrees
*   - 150 degrees
*   - 165 degrees
*   - 180 degrees
*   - 195 degrees
*   - 210 degrees
*   - 225 degrees
*   - 240 degrees
*   - 255 degrees
*   - 270 degrees
*   - 285 degrees
*   - 300 degrees
*   - 315 degrees
*   - 330 degrees
*   - 345 degrees
*
* This program will also log the temperature, in degrees Celsius,
* at which the data was captured. 
*
* Author: Ethan Garnier
* Date: November 25, 2022
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Set the delay between fresh samples
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Number of data points we capture at each test
#define DATA_POINTS 100

// Digital input pin for button
const int buttonPin = 22;
int buttonState = 0;

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

void TestHeading(uint16_t expectedHeading)
{
  Serial.print("-----------\n\n");
  Serial.print("Orient IMU to heading ");
  Serial.print(expectedHeading);
  Serial.print(" degrees.\nPress the pushbutton when ready to capture data points\n");

  // loop until button state is low
  while (buttonState == LOW){}

  // capture DATA_POINTS data points
  for (int i = 0; i < DATA_POINTS; i++)
  {
    // Get a new sensor event
    sensors_event_t event;
    bno.getEvent(&event);

    // Output data point    
    Serial.print("Data Point ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(event.orientation.x, 4);
    Serial.println();

    // Wait the specified delay before requesting next data
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
  Serial.print("-----------\n\n");
}

void setup() 
{
  Serial.begin(9600);
  Serial.print("BNO055 Heading Reader\n\n"); 

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
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

  int8_t opTemp = bno.getTemp();   
  Serial.print("Operating Temperature: ");
  Serial.print(opTemp);
  Serial.println(" C");
  Serial.println("");

  // NOTE: WE ARE NOT PLACING THE BELOW CODE IN 
  // THE LOOP METHOD BECAUSE WE DO NOT WANT TO RUN
  // THIS TEST INFINITELY.

  // We are increasing the expected heading value
  // by 15 on each test.
  uint16_t heading;
  for (heading = 0; heading < 360; heading += 15)
  {
    TestHeading(heading);
  }

  Serial.print("\n\nTEST FINISHED\n\n");  
}

void loop() 
{ 
}