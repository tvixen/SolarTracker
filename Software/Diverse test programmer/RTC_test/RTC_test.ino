//*************************************************************************
// Version: Arduino IDE 1.0.6
//
// Author: Tim Milgart
//
// Start Date: 10.10.2014
// License: Tim Milgart
// Update: 05-02-2015


// Time lib
//#include <time_old.h>  //Added 24/4-2017 da time.h ikke er blevet opdateret af udvikleren.
#include <TimeLib.h>     //TimeLib kan nu det samme som time.h
// RTC lib
#include "DS1302RTC.h"


//Global Date Variables
byte BadRTC     = 0;        // Check if RTC i working or not working =0
bool Sommertid  = false;   



// Define pin RTC1302 
// DS1302:         CE pin    -> Arduino Digital 2  RST  MEGA 39
// DS1302:         I/O pin   -> Arduino Digital 3  DAT  MEGA 38
// DS1302:         SCLK pin  -> Arduino Digital 4  CLK  MEGA 40
// DS1302:         Remember 5V and not 3,3V
// Init the DS1302 
// Set pins:  RST, DAT,CLK
DS1302RTC RTC(10, 11, 12); 


//********************
//Initialize and setup
//********************
void setup(void) 
{
  //Initialize ports and variables  
  //pinMode(53, OUTPUT);  // SPI CS bruges ikke, men skal være sat til OUTPUT 

  tmElements_t tm;   //Get the whole string from RTC      

  Serial.begin(115200);  


  // initialize watch: Get the unix time and print it
  Serial.print("RTC.get  UNIX Time: ");
  Serial.print(RTC.get());
  
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet)  // test om finbindelse er opnået
     Serial.println("Unable to sync with the RTC");
  else
     Serial.println("RTC has set the system time");   


  if (! RTC.read(tm))  // Check if RTC is alive
   {
     Serial.print("  Time = ");
     print2digits(tm.Hour);
     Serial.write(':');
     print2digits(tm.Minute);
     Serial.write(':');
     print2digits(tm.Second);
     Serial.print(", Date (D/M/Y) = ");
     Serial.print(tm.Day);
     Serial.write('/');
     Serial.print(tm.Month);
     Serial.write('/');
     Serial.print(tmYearToCalendar(tm.Year));
     Serial.print(", DoW = ");
     Serial.print(tm.Wday);
     Serial.println();
     Serial.println("RTC Alive");
     BadRTC=0;  //RTC i alive jubii
   } 
  else   // if not alive print error message  
   {
     Serial.println("DS1302 read error!  Please check the RTC circuitry.");
     Serial.println();
     BadRTC=1;
   }
  
  if ( RTC.readRTC(DS1302_TRICKLE) == DS1302_TCR_INI )   //Check the trinkler if its disabled. Can only be enabled if the battery is LiPo.
     Serial.println("Trickle Charger have power-on initial state! (DISABLED)");
  else
     Serial.println("Warning! Charger may be enabled!");


   

  //Set watch
  time_t myTime;
  myTime = RTC.get();



  // init startdate
  time_t t = now();
  minute(t);  // returns the minute for the given time t 

  Serial.println("Ver RTC  test til SolTracker_til_Panel_ver 2.2.1");

  Serial.println("SETUP DONE !");
 //setTime(16, 28, 0, day(t), month(t), year(t)); 
 //setTime(17, 30, 0, 7, 5, 16); 
 //RTC.set(now());                     // set the RTC from the system time
}


// ************************************
// Put a "0" in front of single digit and print it on serial monitor.
// ************************************
void print2digits(int number) 
{
  if (number >= 0 && number < 10)
    Serial.write('0');
  Serial.print(number);
}


//**********************************
// The loop where everything happens
//**********************************
void loop(void) 
{


  time_t t = now();
if (minute(t)==0)
   setTime(17, 30, 1, 7, 5, 16);  
   
  Serial.print("Tiden er : ");
    Serial.print(hour(t));  Serial.print(":");       
    Serial.print(minute(t));Serial.print(":");        
    Serial.println(second(t)); 
//setTime(17, 30, 0, 7, 5, 16); 
 //RTC.set(now()); 


delay(1000);


}
