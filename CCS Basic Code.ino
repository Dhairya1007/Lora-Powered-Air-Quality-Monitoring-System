/******************************************************************************
  Read basic CO2 and TVOCs

  Marshall Taylor @ SparkFun Electronics
  Nathan Seidle @ SparkFun Electronics

  April 4, 2017

  https://github.com/sparkfun/CCS811_Air_Quality_Breakout
  https://github.com/sparkfun/SparkFun_CCS811_Arduino_Library

  Read the TVOC and CO2 values from the SparkFun CSS811 breakout board

  A new sensor requires at 48-burn in. Once burned in a sensor requires
  20 minutes of run in before readings are considered good.

  Hardware Connections (Breakoutboard to Arduino):
  3.3V to 3.3V pin
  GND to GND pin
  SDA to A4
  SCL to A5

******************************************************************************/
#include <Wire.h>

#include "SparkFunCCS811.h" //Click here to get the library: http://librarymanager/All#SparkFun_CCS811

//#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR 0x5A //Alternate I2C Address

CCS811 mySensor(CCS811_ADDR);
long co2,tvoc;
uint8_t data1[10];
uint8_t data2[10];

void setup()
{
  Serial.begin(115200);
  Serial.println("CCS811 Basic Example");

  Wire.begin(); //Inialize I2C Hardware

  if (mySensor.begin() == false)
  {
    Serial.print("CCS811 error. Please check wiring. Freezing...");
    while (1)
      ;
  }
}

void loop()
{
  //Check to see if data is ready with .dataAvailable()
  if (mySensor.dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    mySensor.readAlgorithmResults();

    
    co2 =  mySensor.getCO2();
    String x = String(co2);

    for(int i = x.length()-1;i>=0;i--)
    {
      data1[i] = co2 % 10;
      co2 /= 10;
    }
    for(int i=0;i<10;i++)
    {
      Serial.println(data1[i]);
    }
    tvoc = mySensor.getTVOC();
    String y = String(tvoc);

    for(int i = y.length()-1;i>=0;i--)
    {
      data2[i] = tvoc % 10;
      tvoc /= 10;
    }
    for(int i=0;i<10;i++)
    {
      Serial.println(data2[i]);
    }
    delay(1000);
    
  }

  delay(10); //Don't spam the I2C bus
}
