
/* Author = helscream (Omer Ikram ul Haq)*/



#include <Wire.h>
#include "compass.h"

void setup(){
  Serial.begin(9600);
  // Serial.print("Setting up I2C ........\n");
  Wire.begin();
  compass_x_offset = 122.17;
  compass_y_offset = 230.08;
  compass_z_offset = 389.85;
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;
  
  /*Run Normal, without debug information  */
  compass_init(2);
  /*Run with debug information*/
    compass_debug = 1;
  /*Run with calibration*/
    compass_offset_calibration(3);

}

void loop(){
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
{Serial.println("Nord");}

if (bearing > 325 && bearing < 360)
{Serial.println("Nord");}

if (bearing > 45 && bearing < 135)
{Serial.println("East");}

if (bearing > 135 && bearing < 225)
{Serial.println("SOUTH");}

if (bearing >= 158 && bearing <= 161)
{Serial.println("DUE SOUTH");}

if (bearing > 225 && bearing < 325)
{Serial.println("West");}
 
  
  delay(1000);
}
