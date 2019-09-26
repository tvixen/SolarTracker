//*************************************************************************
// Version: Arduino IDE 1.6.8
//
// Author: Tim Milgart
//
// Start Date: 06.05.2016
// License: Tim Milgart
// Update: 06-05-2016
// Har taget den eksisterende kode fra SolTracker_til_Panel_ver_2.2.2 og rettet til, så den passer til mini panelet til carporten.
// Update 06/05-2016
// Jeg skriver koden til en UNO i stedet for en MEGA, da der kun skal bruges 2 analoge indgange og 7 digitale in/out
// Update 13/05-2016
// Har tilføjet stop motor under "if (LightAverage < 550 )" og debug printer kun hvert 1.sekund nu.
// Update 21/05-2016
// Update Version 1.0.1
// Har tilføjet RTC modul og flyttet data linjerne 1,2,3 til 3,4,5. Samt rykket de andre tilsvarende.
// Update 22/05-2016
// Har rettet i koden til " if (LightAverage < 500 ) " så tiden bliver sat inden den hopper ud af løkken.
// Update 31/05-2016
// Har rettet i koden til " if ((lightSensorEastValue < " så Panelet stopper når end schwitches bliver aktiveret 
// Update 04/06-2016
// Dette er en test program. Og kan kun bruges til test, da alle input er blevet gjort faste.

// Time lib
#include "Time.h"
// RTC lib
#include "DS1302RTC.h"
// Show memory lib
//#include "MemoryFree.h"
// Interrupt lib
#include <SPI.h>



// Define pin RTC1302 Real Time Clock
// DS1302:         CE pin    -> Arduino Digital 2  RST  MEGA 39
// DS1302:         I/O pin   -> Arduino Digital 3  DAT  MEGA 38
// DS1302:         SCLK pin  -> Arduino Digital 4  CLK  MEGA 40
// DS1302:         Remember 5V and not 3,3V
// Init the DS1302 
// Set pins:  RST, DAT,CLK
DS1302RTC RTC(10, 11, 12); // 0,1 kan ikke bruges hvis der bruges serial data til PC (debug)


//Define solartracker ports, variables and static
//Digital ports D0-D13
// 0 , 1 reserved for serial communication/Debug info
const byte dipSwitchWest     = 5;     // Endstop West
const byte dipSwitchEast     = 6;     // Endstop East
const byte MotorMoveEastWest = 8;     // relæ motor1 øst/vest styring af solpanelet
const byte MotorEWOn         = 9;     // relæ motor1 on/off til øst/vest 
const byte LED               = 13;


//Global Date Variables
int NextTrackOffset                                 = 0;  // Indeholder realtime minutter, der skal trækkes fra waitMinutesToNextTrackUpdate så tiden blir 0. Og der kan opdateres.
int waitMinutesToNextTrackUpdate                    = 2;  //This value is in minutes 1 equal one minut 5 eual 5 minuttes. In my application it is Set to 3 or 5 minutes
int NextTrackOffsetANDwaitMinutesToNextTrackUpdate  = 0;  //NextTrackOffset + waitMinutesToNextTrackUpdate 
int SetdifferenceEastWest                           = 10; //Tolerancen på hvornår panelet skal dreje (hvis sensorværdien er mere end 40 i forskel fra East til West) 
byte BadRTC                                         = 0;  // Check if RTC i working or not working =0
int prevsec                                         = 0;  // indeholder værdien af de tidligere sekunder                                 
bool Sommertid  = false;                                  // Bruges til kontrol af sommer/vinter tid    

//Global Bolean 
bool East = HIGH; //Active high
bool West = LOW;  //Active Higi
bool Run  = HIGH; //Active High
bool Stop = LOW;  //Active LOW
bool dipSwitchWestState = 0;
bool dipSwitchEastState = 0;
long int count = 0; // for test purppose

//************************************
//Initialize and setup
//No serialprint in setup procedure
//************************************
void setup(void) 
{
  //Initialize ports and variables  
  analogReference(DEFAULT); //INTERNAL1V1 = 1.1V INTERNAL2V56 = 2.56V , DEFAULT = 5 volt


  pinMode(MotorEWOn,         OUTPUT); // Motor1 on-off
  pinMode(MotorMoveEastWest, OUTPUT); // East 0, West 1
  pinMode(dipSwitchWest, INPUT);    
  pinMode(dipSwitchEast, INPUT);    

  pinMode(LED,         OUTPUT); 

  StopMotorEW();

  tmElements_t tm;   //Get the whole string from RTC      



  //  initialize serial communication at 115200 bits per second:
  //  Serial.begin(57600);
  //  Serial.begin(9600);
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


  // init startdate
  time_t t = now();
  minute(t);  // returns the minute for the given time t 
  NextTrackOffset = minute(t);
  Serial.println("TEST Ver SolTracker_til_MiniPanel_ver 1.0.1");
  
  // kunstig dato. Kun til test
  //  setTime(20, 32, 50, 18, 6, 2016); 

  digitalWrite(LED, Stop); // test led

  Serial.println("SETUP DONE !");

  //waitMinutesToNextTrackUpdate = 1;
  dipSwitchWestState = 0;
  dipSwitchEastState = 0;
}


// ************************************
// Can do a software reset of the Arduino print
// ************************************

void SoftReset()
{
asm volatile ("  jmp 0");  // Restarts program from beginning but does not reset the peripherals and registers
}


// ************************************
// Get the lightsensor value on 4 Analog pins
// and save them in Local variables
// ************************************
void ReadLightSensorEastWest()
{

int lightSensorWest = A0;  // 
int lightSensorEast = A1;  //



//Global Light Variables
int lightSensorEastValue    = 750;
int lightSensorWestValue    = 763;
int LightAverage            = 0;           //Lavet global for at kunne læse den i displayet


bool EWinPosition=false;                   //Bool værdi til at resætte tiden hvis panelet har været i position


int sampletime=10;
   for(int i=0;i<sampletime;i++)  // taking 50-100 samples from sensors with a inerval of 2msec and then average the samples data collected
    {
      lightSensorEastValue   += analogRead(lightSensorEast);    //read the voltage from the sensor
      lightSensorWestValue   += analogRead(lightSensorWest);
      delay(2);
    }
   lightSensorEastValue=lightSensorEastValue/sampletime;   // hiv gennemsnittet så blir det lidt mere præcist
   lightSensorWestValue=lightSensorWestValue/sampletime;

lightSensorEastValue    = 750;
lightSensorWestValue    = 763;


   // Avarage fra alle sensorer, giver tal der kan måle om lyset er for svagt. 300= en gråvejrsdag i november uden sol og bare skyer.
   LightAverage = (lightSensorEastValue + lightSensorWestValue ) / 2; 


   int DifHoizontal = lightSensorEastValue - lightSensorWestValue;     // the difference between the 2 sunvalues East and West
   
   int differenceEastWest = abs(DifHoizontal);                             // 'abs' is the absolute value, the result is always positive.
   
   time_t t = now();  // skal deklareres ellers kan den ikke se tiden 

   /* Forklaring af tids-variabler                                     */
   // NextTrackOffset                                = selve real tiden
   // waitMinutesToNextTrackUpdate                   = tiden jeg kan indstille fra 1-15 minutter
   // NextTrackOffsetANDwaitMinutesToNextTrackUpdate = tiden der er til næste track update ( realtiden lagt sammen med tiden jeg selv kan indstille )

   // Kontrol af tiden, om den ryger forbi 60
   NextTrackOffsetANDwaitMinutesToNextTrackUpdate = NextTrackOffset+waitMinutesToNextTrackUpdate;
   if (NextTrackOffsetANDwaitMinutesToNextTrackUpdate > 59)  // sker ved f.eks NextTrackOffset=57 og waitMinutesToNextTrackUpdate er 3 minutter
     NextTrackOffsetANDwaitMinutesToNextTrackUpdate=NextTrackOffsetANDwaitMinutesToNextTrackUpdate-60; // ved at trække 60 fra, blir resultatet 0 eller mere end 0

if ((minute(t) == 55 )&& (second(t)==1)) 
   EWinPosition=true;

if ((minute(t) == 59 )&& (second(t)==1)) 
   EWinPosition=true;  

if ((minute(t) == 3) && (second(t)==1))
   EWinPosition=true;  



   
  /***************************DEBUG INFO  ***************/
  if (((second(t) % 1 == 0) && (prevsec != second(t))))  //hvert sekund ..hvis den er spot on
     { 
 // TIMESTAMP
     Serial.println();                 
     Serial.println();       
     Serial.print("Tiden(t)= ");       
     Serial.print(hour(t));   Serial.print(":");       
     Serial.print(minute(t)); Serial.print(":");        
     Serial.println(second(t)); 
     
 //show SensorThreshold
      Serial.println();                 
      Serial.print("Minute: ");  Serial.println(minute(t));          
      Serial.print("NextTrackOffset = " ); Serial.println(NextTrackOffset);     
      Serial.print("NextTrackupdate + NextTrackOffset: "); Serial.println(waitMinutesToNextTrackUpdate+NextTrackOffset);   
      Serial.print("NextTrackOffsetANDwaitMinutesToNextTrackUpdate: ");  Serial.println(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
      Serial.println("When NextTrackOffsetANDwaitMinutesToNextTrackUpdate are the same as minute, the Panel will go West or East ");
      Serial.println("*****************************************************************************");            
      Serial.print("LightAverage is ");  Serial.println(LightAverage);      
      Serial.print("East: ");  Serial.println(lightSensorEastValue);
      Serial.print("West: ");  Serial.println(lightSensorWestValue);
      Serial.print("SetdifferenceEastWest: ");  Serial.println(SetdifferenceEastWest);
      Serial.print("Real differenceEastWes: ");  Serial.println(differenceEastWest);
      Serial.println();            
      Serial.print("dipSwitchEaseState: ");  Serial.println(digitalRead(dipSwitchEast));   
      Serial.print("dipSwitchWestState: ");  Serial.println(digitalRead(dipSwitchWest));         
      Serial.print("count: ");  Serial.println(count);         
      Serial.println("*****************************************************************************");                  
      prevsec=(second(t)); 
     }


   if (LightAverage < 550 ) 
    {
     StopMotorEW();       // Stop motor
     ResetRelayEW();      // Reset relæ så det ikke trækker strøm hele tiden.    

     /***************************DEBUG INFO  ***************/
     Serial.println("***************************DEBUG INFO BEFORE EXIT ***************"); 
     Serial.println();
     Serial.print("No sun light,  Powersave mode. Exit Calculations. LightAverage:");Serial.println(LightAverage);
     Serial.println(" "); 
     Serial.print("differencen East/West was ");   Serial.println(differenceEastWest);        
     
     Serial.println("----------------------");
     Serial.print("Realtime = ");                             Serial.print(hour(t)); Serial.print(":");Serial.print(minute(t)); Serial.print(":"); Serial.println(second(t)); 
     Serial.print("NextTrackOffset = ");                      Serial.println(NextTrackOffset);
     Serial.print("waitMinutesToNextTrackUpdate = ");         Serial.println(waitMinutesToNextTrackUpdate);
     Serial.print("NextTrackOffsetAND = ");                   Serial.println(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
     Serial.println(" "); 
     Serial.print("When minute(t) ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print("NextTrackOffset + waitMinutesToNextTrackUpdate "); Serial.print(NextTrackOffset+waitMinutesToNextTrackUpdate);  Serial.println(" then UPDATE  *** ");
     Serial.print("When minute(t) ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print("NextTrackOffsetANDwaitMinutesToNextTrackUpdate "); Serial.print(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);  Serial.println(" the same  *** ");
     Serial.println(" "); 
     Serial.println("Too little light .... now EXIT  ");     
     Serial.println(" ");     
     Serial.println(" ");   

    //  husk at sætte timeren inden exit, ellers går det galt med tiden
     time_t t = now();    
     NextTrackOffset = minute(t);     
  
 
     return;          //hop tilbage til hvor den blev kaldt
    }  


 /********* Her kommer den rigtige beregning på panelets hældning *********************/

  
    
  if ((lightSensorEastValue < lightSensorWestValue) && (differenceEastWest > SetdifferenceEastWest)) //omkring 40 så kører den ikke så ofte
    {
//     Serial.println("(lightSensorEast < lightSensorWest ...jeg justerer West.. hvis tiden er korrekt");
     time_t t = now();
     if ((NextTrackOffsetANDwaitMinutesToNextTrackUpdate) == (minute(t)))  //Hvis NextTrackOffset og waitminuttesnexttrackupdate er det samme som tiden, så go west
      {
       // count = count +1;
        goToWest();
        Serial.println("Automatic, Going to West");
      }
    }
  else if ((lightSensorEastValue > lightSensorWestValue) && (differenceEastWest > SetdifferenceEastWest))
   {
//    Serial.println("(lightSensorEast > lightSensorWest ... jeg justerer East....hvis tiden er korrekt)");
     time_t t = now();
     if ((NextTrackOffsetANDwaitMinutesToNextTrackUpdate) == (minute(t)))   //Hvis NextTrackOffset og waitminuttesnexttrackupdate er det samme som tiden, så go east
     {
        goToEast();
        Serial.println("Automatic, Going to East");       
     } 
   }
  else if (differenceEastWest <= SetdifferenceEastWest)  // hvis forskellen på de 2 værdier er mellem 0 og 60 så er de så godt som lige, og nexttrack kan udregnes
   {
     EWinPosition=true; //bit sat til Nexttime ok
     //Serial.println("YES, E/W is in position ");
     //Serial.print("differencen East/West er ");  Serial.println(differenceEastWest);        
     StopMotorEW();                          // ingen update når sensor er lige og NextTrackOffset bliver = time now minutter
     ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.        
   }// else if


  // Nexttrack=time(t) ...hvis begge er i rette position
  if (EWinPosition )
  { 
    time_t t = now();    
    NextTrackOffset = minute(t);     
    //Serial.println(" ");  
    //Serial.print("NextTrackOffset er nu RESAT og ny periode af NextTrack kan begynde");      
    //Serial.println(count);  
    ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.        
    count = count +1;                    // 9657 gange læses den uden delay. 95 gange med delay(10)
  }


/***************************DEBUG INFO  **************
    Serial.println(" "); 
    Serial.println(" "); 
     Serial.println("--------The calc is over--------------");
    Serial.print("differencen East/West is ");
    Serial.println(differenceEastWest);        

    Serial.println("----------------------");
    Serial.print("Realtime = ");                                       Serial.println(hour(t)); Serial.print(":"); Serial.print(minute(t)); Serial.print(":"); Serial.println(second(t)); 
    Serial.print("NextTrackOffset = ");                                Serial.println(NextTrackOffset);
    Serial.print("waitMinutesToNextTrackUpdate = ");                   Serial.println(waitMinutesToNextTrackUpdate);
    Serial.print("NextTrackOffsetANDwaitMinutesToNextTrackUpdate = "); Serial.println(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
    Serial.println(" "); 
    Serial.print("When ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print(NextTrackOffset+waitMinutesToNextTrackUpdate);  Serial.println(" then UPDATE  *** ");
    Serial.print("When ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);  Serial.println(" Should be same number  *** ");
    Serial.println(" "); 
    Serial.println(" ");     


Serial.println("----------------------------------------------SLUT ReadLightValues----------------------------------- ");  //hvis ikke denne er her, vil uret ikke vises   
Serial.println();
Serial.println();
*/
}



// ******************************************
// Reset Relæ så de ikke står med kontakterne
// sluttet og bruger strøm
// ******************************************
void ResetRelayEW()
{ 
  digitalWrite(MotorMoveEastWest, West);  //Relæ sættes i off position, så den ikke trækker strøm i standby
  digitalWrite(MotorEWOn, Run);           //Relæ Motor1 kør
  digitalWrite(MotorEWOn, Stop);          //Low = stop
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
// start up. Go Home to East 
//**********************************
void goHomeToEast()
{
  digitalWrite(MotorMoveEastWest, East);        // Vend Relæ East/West mod Øst
  digitalWrite(MotorEWOn, Run);                 // Start MotorRelæ EW, Panel kører
  dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchEastState == 1)
  {
   StopMotorEW();
   ResetRelayEW();                                // Reset relæ så det ikke trækker strøm hele tiden.            
  }

}

//********End goHomeToWest********************

//**********************************
// MotorEastWest Goto West
//**********************************
void goToWest()
{
  digitalWrite(MotorMoveEastWest, West);           // Relæ sættes til at køre vest på 
  digitalWrite(MotorEWOn, Run);                    // Motor kør
  dipSwitchWestState = digitalRead(dipSwitchWest); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchWestState == 1)
    {
      StopMotorEW();
      ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.    
    }
}
//*******End goToWset*********************

//**********************************
// MotorEastWest Goto East
//**********************************
void goToEast()
{
  digitalWrite(MotorMoveEastWest, East);  
  digitalWrite(MotorEWOn, Run); 
  dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchEastState == 1)
  {
    StopMotorEW();
    ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }
}
//******End goToEast**********************



//**********************************
// Stop motorEastWest
//**********************************
void StopMotorEW()
{
  digitalWrite(MotorEWOn, Stop); //Low = stop
}
//*********End stopMotor*******************


//**********************************
// The loop where everything happens
//**********************************
void loop(void) 
{
    
  time_t t = now();
  dipSwitchWestState = digitalRead(dipSwitchWest); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchWestState == 0)                     // Hvis dipswitz er trykket ned så spring ReadLightSensor over
    ReadLightSensorEastWest();                     // Læser East/West. Motoren blir styret af solen her. 
  else 
    goToEast();

  //hvis tid er ml 20:00 og 00:01 så gå hjem til øst.
  if (hour(t)==21 && minute(t) == 0)     //Reset hver 12 time               
     {
       dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch west 1 = pressed = end stop
       while(dipSwitchEastState == 0)                   // Alting stopper når denne while løkke kører. Kører indtil kontakten bliver High
         goHomeToEast();                                // Er klokken mere end 21:00 så kør hjem mød øst. Og reset relæerne.

       StopMotorEW();                                   // Jeg vil være helt sikker på at motoren stopper.  
       ResetRelayEW();                                  // Reset relæ så det ikke trækker strøm hele tiden.            
      
     }

//delay(10);                                              // delay(10) er en faktor 100 x (reel fra 43 til 30, når samples er med i programmet)
}

