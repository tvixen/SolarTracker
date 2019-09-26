//*************************************************************************
// Version: Arduino IDE 1.0.6 + 1.8.2
//
// Author: Tim Milgart
//
// Start Date: 10.10.2014
// License: Tim Milgart
// Update: 24-04-2017
// Tester bare time.h biblioteket
// samme kode som RTC_test, men uden RTC lib DS1302RTC.h
// Koden går bare ud på om den kan compiles eller ej, ikke om resultatet i koden.
//*************************************************************************


// Time lib           //Gammelt  
#include <TimeLib.h>  //Added 24/4-2017 da time.h ikke er blevet opdateret af udvikleren.



//Global Date Variables
byte BadRTC     = 0;        // Check if RTC i working or not working =0
bool Sommertid  = false;   

//********************
//Initialize and setup
//********************
void setup(void) 
{

  tmElements_t tm;   //Get the whole string from RTC      

  Serial.begin(115200);  

  // initialize watch: Get the unix time and print it
  Serial.print("RTC.get  UNIX Time: ");

  

  if(timeStatus()!= timeSet)  // test om finbindelse er opnået
     Serial.println("Unable to sync with the RTC");
  else
     Serial.println("RTC has set the system time");   


//  if (! RTC.read(tm))  // Check if RTC is alive
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
//  else   // if not alive print error message  
   {
     Serial.println("DS1302 read error!  Please check the RTC circuitry.");
     Serial.println();
     BadRTC=1;
   }
  


   

  //Set watch
  time_t myTime;




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
//   setTime(23, 59, 59, day(t), month(t), 2015); 
  Serial.print("Tiden er : ");
    Serial.print(hour(t));  Serial.print(":");       
    Serial.print(minute(t));Serial.print(":");        
    Serial.println(second(t)); 
delay(1000);
}
