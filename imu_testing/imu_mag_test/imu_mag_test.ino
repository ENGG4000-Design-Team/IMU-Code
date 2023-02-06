#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Magnetometer Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  // Get raw vector data from magnetometer
  // unit is micro Teslas
  imu::Vector<3> magVect = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(magVect.x());
  Serial.print(" Y: ");
  Serial.print(magVect.y());
  Serial.print(" Z: ");
  Serial.print(magVect.z());
  Serial.print("\n");

  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}