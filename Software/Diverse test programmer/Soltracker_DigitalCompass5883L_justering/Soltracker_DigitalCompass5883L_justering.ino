//*************************************************************************
// Version: Arduino IDE 1.6.9
// Compass.h Author: helscream (Omer Ikram ul Haq)
// http://hobbylogs.me.pn/?p=17
// https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration
//
// Code Modified by: = Tim Milgart
// Start Date: 02.07.2016
// Update: 03-07-2016
// Added Text when bearing is within the desired angle
// Update 03/07-2016
// Added Text when bearing due south

#include <Wire.h>
#include "compass.h"


//************************************
//Initialize and setup
//No serialprint in setup procedure
//************************************
void setup()
{
  Serial.begin(115200);
  //Serial.begin(9600);
  Wire.begin();

  //Offset x  = -4.23 mG
  //Offset y  = -1.28 mG
  //Offset z  = 403.99 mG

  // Mine gamle offset værdier efter calibration
  //compass_x_offset = -4.23;
  //compass_y_offset = -1.28;
  //compass_z_offset = 403.99;

  //Nye Aflæste værdier 30/11-2016  efter calibration
  compass_x_offset = 46.80;
  compass_y_offset = 171.70;
  compass_z_offset = 393.89;


  //compass_x_offset = 122.17;
  //compass_y_offset = 230.08;
  //compass_z_offset = 389.85;
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;
  
  /*Run Normal, without debug information  */
  compass_init(2);
  /*Run with debug information*/
  //  compass_debug = 1;
  /*Run with calibration*/
  //  compass_offset_calibration(3); // On a plane surface. Turn the compass print around the clock 360 degrees, with 1 sek interval. 
                                     // Start from North and go to east, then south, then west, then north.
}
//**********************************
// The loop where everything happens
//**********************************
void loop()
{
  
  Serial.println(" ");
  Serial.println("compass_scalled_reading");
  compass_scalled_reading();
   
  Serial.print("x = ");
  Serial.println(compass_x_scalled);
  Serial.print("y = ");
  Serial.println(compass_y_scalled);
  Serial.print("z = ");
  Serial.println(compass_z_scalled);
  

  compass_heading();
  Serial.print ("Heading angle = ");
  Serial.print (bearing);
  Serial.println(" Degree");

  if (bearing > 0 && bearing < 45)
   {
    Serial.println("North");
   }

  if (bearing > 325 && bearing < 360)
   {
    Serial.println("North");
   }

  if (bearing > 45 && bearing < 135)
   {
    Serial.println("East");
   }

  if (bearing > 135 && bearing < 225)
   {
    Serial.println("South");
   }

  if (bearing >= 177 && bearing <= 183)
   {
    Serial.println("DUE SOUTH");
   }

  if (bearing > 225 && bearing < 325)
  {
   Serial.println("West");
  }
  Serial.println();
  Serial.println();
  delay(1000);
}
