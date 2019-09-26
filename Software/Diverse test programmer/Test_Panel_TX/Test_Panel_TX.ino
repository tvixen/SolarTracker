//*************************************************************************
// Arduino IDE Version: 1.8.5
//
// Author: Tim Milgart
//
// Program for: Solar Panel. 
// Function:    Test TX/RX function
//
// Start Date: 07.12.2018
// License: Tim Milgart
//*************************************************************************

// sakset fra SolTracker_til_NytPanel_2.9_job


//Intern Time regnskab
#include "TimeLib.h"
//Radio module NRF24L01+
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <avr/wdt.h>          // watchdog

//these 3 is for the new RTC
#include <Wire.h>            //for I2C
#include <RTClibExtended.h>  //for RTC DS3231
#include <LowPower.h>        //for interrupt and sleep  

// Define NEW DS3231 Real Time Clock
// DS3231:         SCL pin    -> Arduino Digital 21 MEGA 21
// DS3231:         CLK pin    -> Arduino Digital 20 MEGA 20
// DS3231:         Remember 5V and not 3,3V
RTC_DS3231 RTC;      //we are using the DS3231 RTC EXTENDED


//Define solartracker ports, variables and static
//Digital ports D0-D13
// 0 , 1 reserved for serial communication/Debug info


const byte HardWareEndStopOverWrite  = 37;     // relæ til overskrive hardware end stop ved høj
const byte MotorMoveEastWest         =  8;     // relæ motor1 øst/vest styring af solpanelet
const byte MotorEWOn                 =  9;     // relæ motor1 on/off til øst/vest 
const byte ReedSwitchEast            =  6;     // Endstop East
const byte ReedSwitchWest            =  5;     // Endstop West
const byte RTCWakeUpPin              =  2;     // use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW. Saving 40mA on Mega alone
const byte EndStopInterruptPin       =  3;     // interrupt 1 (pin 3) attachInterrupt(digitalPinToInterrupt(pin), ISR, mode); (recommended)   
const byte BeepPin                   =  4;     // Lille piezo højtaler der kan afgive lyde   
const byte RadioOn_Count_time        = 30;     // keeps the ReadLightSensorEastWest() out of loop for 30 sec.

//Global Date Variables
int NextTrackOffset                                 = 0;  // Indeholder realtime minutter, der skal trækkes fra waitMinutesToNextTrackUpdate så tiden blir 0. Og der kan opdateres.
int waitMinutesToNextTrackUpdate                    = 2;  // This value is in minutes 1 equal one minut 5 eual 5 minuttes. In my application it is Set to 3 or 5 minutes
int NextTrackOffsetANDwaitMinutesToNextTrackUpdate  = 0;  // NextTrackOffset + waitMinutesToNextTrackUpdate 
int SetdifferenceEastWest                           = 30; // Tolerancen på hvornår panelet skal dreje (hvis sensorværdien er mere end 40 i forskel fra East til West) 
byte BadRTC                                         = 0;  // Check if RTC i working or not working =0
int prevsec                                         = 0;  // indeholder værdien af de tidligere sekunder                                 

int incomingByte                                    = 53; // Serial read, til at styre E/W motor manuelt  53=ascii 5
int NoLink                                          = 0;  // Optæller link fejl
byte LightAverageDebugInfo                          = 1;  // Debuginfo er online. Findes under ReadLightSensorEastWest
byte RadioOn_count                                  = RadioOn_Count_time;  // Counter if the radio is on

//Global Boolean 
bool East = HIGH; //Active high
bool West = LOW;  //Active Higi
bool Run  = HIGH; //Active High
bool Stop = LOW;  //Active LOW
bool ReedSwitchWestState = 0;
bool ReedSwitchEastState = 0;
//bool Sommertid  = false;                                // Bruges til kontrol af sommer/vinter tid    
//bool RadioLink_is_UP = false;                           // Boolean om radio linken er forbundet. Skal bruges til at springe ReadLightSensorEastWest over, så relæ ikke klikker når man kører manuelt.
volatile bool EndSwitchIsReached = false;                 // Boolean i interrupr afgør om endswitch er nået, så blir den true(hvis input er høj)


//Analog ports A0-A5
int lightSensorWest = A0;  // 
int lightSensorEast = A1;  //
// +5V Hvid+Orange   
// GND Brun


//Global Light Variables. Also used in RF24
int lightSensorEastValue    = 0;
int lightSensorWestValue    = 0;
int LightAverage            = 0;           //Lavet global for at kunne læse den i displayet
int VoltageEast             = 0;           //Viser spændingen på porten
int VoltageWest             = 0;           //Viser spændingen på porten

bool EWinPosition=false;                   //Bool værdi til at resætte tiden hvis panelet har været i position


//Global Bolean 
bool ButtonMotoEWOnState      = 0;
bool ButtonMotorMoveEastState = 0;
bool ButtonMotorMoveWestState = 0;

//String variables
byte drawString         = 0;  //used for different type of strings to be displayed, when different actions accure

// Set up nRF24L01 radio on SPI bus plus pins 9 & 53 kan godt ændres til pin 10 eller andet.
//PRINT LAYOUT for Mega:
//1= GND  2=VCC (3,3Volt)
//3=CE(45)    4=CSN(49)
//5=CSK(52)   6=MOSI(51)
//7=MISO(50)  8=IRQ(nc)
RF24 radio(45,49); //(CE,CSN)
//const uint64_t pipes[2] = { 0xDEDEDEDEE7LL, 0xDEDEDEDEE9LL }; // kode til de 2 moduler (gamle værdier)
const uint64_t pipes[2] = { 0xDEDEDEDEE1LL, 0xDEDEDEDEE2LL };   // Write, Read
char *val;
char SendPayload[32] = "";
char RecvPayload[32] = "";
int crc=0;


// This variable is made volatile because it is changed inside
// an interrupt function
// volatile int f_wdt=1; kan ikke se den bruges
// bool MorgenStart = LOW;    //bruges til at starte forfra i main loop

//************************************
//Initialize and etup procedure
//
//************************************
void setup(void) 
{
  wdt_disable();                  // no watchdog yet
  //Initialize ports and variables  
  analogReference(DEFAULT); //INTERNAL1V1 = 1.1V INTERNAL2V56 = 2.56V , DEFAULT = 5 volt


  pinMode(MotorEWOn,                   OUTPUT); // Motor1 on-off
  pinMode(MotorMoveEastWest,           OUTPUT); // East 0, West 1
  pinMode(ReedSwitchWest,                INPUT); // Stop kontakt for West   
  pinMode(ReedSwitchEast,                INPUT); // Stop kontakt for East   
  pinMode(53,                          OUTPUT); // SPI CS bruges ikke, men skal være sat til OUTPUT
  pinMode(HardWareEndStopOverWrite,    OUTPUT); // Kan overskrive de 2 hardware endstops ved høj
  pinMode(RTCWakeUpPin,                 INPUT); // Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(EndStopInterruptPin,          INPUT); // Set pin D3, the switches to this pin, so it will act instantly and not after 1-2 sec
//  attachInterrupt(digitalPinToInterrupt(EndStopInterruptPin), InterruptStopMotor, HIGH);

  Serial.begin(115200);  
//  StopMotorEW();
//  HardWareEndStop(Stop);                //Motor kan kun køre til end stop. HardWareEndStop kan overskrive de 2 end stops, hvis den sættes til høj/Run
  EndSwitchIsReached = false;
  


  
  Serial.println("Version  Test_Panel_TX");
  Serial.println("SolTracker will NOT work with USB power only!  Missing 3 volt supply");
  delay(1000);  //så teksten kan læses
  Serial.println();
  Serial.println();

  ReedSwitchWestState = 0;
  ReedSwitchEastState = 0;


  drawString = 0; // nulstiller drawString så den er klar til nye actions der skal op på skærmen.  

  //NRF24L01+ begynder init her    
  printf_begin();

  radio.begin();
  radio.setDataRate(RF24_250KBPS); //speed  RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.setPALevel(RF24_PA_HIGH);  //Set Power Amplifier (PA) level to one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setChannel(125);           //channel  Which RF channel to communicate on, 0-127
  
  radio.enableDynamicPayloads();
  radio.setRetries(5,15);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableAckPayload();

  // Skal se sådan ud når Panel er RX
  radio.openWritingPipe(pipes[0]);    // Open different pipes when writing. Write on pipe 0, address 0
  radio.openReadingPipe(0,pipes[1]);  // Read on pipe 0, as address 1
  //RX_ADDR_P0-1   = 0xdedededee7 0xc2c2c2c2c2
  //TX_ADDR   = 0xdedededee9


  radio.startListening();
  radio.printDetails();
  
     
  Serial.println("Radio setup DONE !");
  //delay(50000);  //Test
  
  wdt_enable(WDTO_4S); //Watchdog set to 4 sec
 
}

//********End Setup********************



//------------------------------------------------------------


// *************CRC Check******************************
// Funktion til hive de sidste check cifre ud af strengen
// og derefter konverterer det til en integer
// ****************************************************
void CRC_Check(int len)
{
 char tmpcrc[5] = ""; 

 // Serial.print("CRC_Check RecvPayload = ");   Serial.println(RecvPayload);
 // Serial.print("DataPayload = ");   Serial.println(DataPayload);

  // Get CRC value from RecvPayload
  for (int i = 0; i < 4; i++) 
   tmpcrc[i]=RecvPayload[len-6+i];

  val = strtok (tmpcrc,",");

  crc = atoi(val);
 // Serial.print("inkomne len= ");   Serial.println(len);
 // Serial.print("Sidste data= ");   Serial.println(tmpcrc);
 // Serial.print("CRC er = ");   Serial.println(crc);
}

// *************PANEL**********************************
// Modtag procedure i panelet. Med 2 forskellige case
// 21=Motordata  26=Dato
// ****************************************************
void NRF24L01_ReceiveCommandFromDisplay()
{
  int len = 0;
  char DataPayload[32] = "";
  int timer    = 0;   
  int minutter = 0;
  int sekunder = 0;
  int dag      = 0;
  int maaned   = 0;
  int aar      = 0;
  int count=0;
  bool ACKdone = false;   
  String DataStringtoSD = "";    
  int okval=0; 
  int Error=0;
  byte ExitTimer=0;

  
 /* if (NoLink > 28 )
    {
      Serial.println("NoLink: Motor blir nu sat til OFF  ");
      ButtonMotoEWOnState      = 1;  // 1=Motor OFF  0=Run ... Hvis radiolink forsvinder forbliver motor off    
      ReadButtons();
      NoLink=0; 
    }
 */ 
//  Serial.println("-------Start Receive Mode ---------------- ");

  bool result = radio.isChipConnected();
  if (!result) 
   {
   Serial.println("No RF24 connection");
   return;
   }
  
  
  if ( radio.available() ) // er der radiolink
   {
     Serial.println("RadioLink = true;");
     while (radio.available())
      {
       Serial.println("while loop... ");
       len = radio.getDynamicPayloadSize();
       radio.read( RecvPayload, len );
      }
//    Serial.println("Radio link");
    RecvPayload[len] = 0;
   } 
 else 
   {  
    if (NoLink == 3000)  // det går pænt hurtigt når serialprint ikke anvendes ...så omkring 1000 skal den ca. være
      {
        Serial.println("Radio link not ready ");
        Serial.println();
        NoLink=0;
        RadioOn_count=0;            //Denne burde gør så der ikke kan auto justeres når RF24 modulet er aktivt på Displayet.

//        RadioLink_is_UP=false;    //Initial setting.  Blir kun høj hvis der er "ægte" data i protokollen.
      }
        NoLink++;
      //  Serial.print("radio link ++  = ");Serial.println(NoLink);
     //   delay(110);    // kan ikke aflæses så mange gange i sekundet, derfor dette delay
     return;
   }

 if ((RadioOn_count<=RadioOn_Count_time)&&(RadioOn_count !=0)) // hvis radiolink er oppe så start tæller, så readlightsensor() holdes ude af loopet i RadioOn_Count_time antal sekunder x 2, da et loop tager 2 sekunder
   RadioOn_count--;

 if (RadioOn_count==0)                  // tæller starter forfra.  Men den havner kun her hvis der ER radiolink. Ellers forbliver den 0
   RadioOn_count=RadioOn_Count_time;
   

 for (int i = 0; i < 32; i++) 
   DataPayload[i]=RecvPayload[i];

  CRC_Check(len); //Kald crc check med længden på RecvPayload
/*
  Serial.println();
  Serial.print("RecvPayload = ");   Serial.println(RecvPayload);
  Serial.print("DataPayload = ");   Serial.println(DataPayload);
  Serial.print("Den nye CRC er nu   = ");   Serial.println(crc);
  Serial.println(); 
 
  Serial.println("------------slut paa 1.st Receive Data Mode delen -------  ");   
  Serial.println();
*/
//delay(10); //Delay i stedet for serial.print

// rem 21/7-2015  time_t t = now();
  
  DataStringtoSD = "";    
 
  if ((crc==1966)||(crc==1280)) //1966 eller 1280
   {  
     // Sender de 4 LightValues til Display
     int lightparameter[5]={9, 9, lightSensorEastValue, lightSensorWestValue, NextTrackOffsetANDwaitMinutesToNextTrackUpdate}; // 4 værdier af sammt type pakkes til een string. og så en NextTrackUpdate
     DataStringtoSD += ",";  // Komma tilføjes
     for (int i = 0; i < 5; i++)     // read 4 parameters and append to the string:
       {
         int datavalue = lightparameter[i];
         DataStringtoSD += String(datavalue);
         if (i < 5) 
           DataStringtoSD += ","; 
       } 
   } // if (crc ==1966)


  if (crc==1281) // 1281 lavet for MiniRemoten
   {  
     // Sender de 4 LightValues til Display
     int lightparameter[5]={9, 9, lightSensorEastValue, lightSensorWestValue, NextTrackOffsetANDwaitMinutesToNextTrackUpdate}; // 4 værdier af sammt type pakkes til een string. og så en NextTrackUpdate
     DataStringtoSD += ",";  // Komma tilføjes
     for (int i = 0; i < 5; i++)     // read 4 parameters and append to the string:
       {
         int datavalue = lightparameter[i];
         DataStringtoSD += String(datavalue);
         if (i < 5) 
           DataStringtoSD += ","; 
       } 
   } // if (crc ==1281)


  if ((crc==1309)||(crc==1310)) //1309 eller 1310  1310 er MiniRemoten
   { 
     // Sender Volt, Amp, Temperatur, DrawString + crc 
     Serial.println("Henter Volt - Amps - Temperature ");
     float parameter[3]={9,9,9}; // sender  9,9,9 da Volt - Amps - Temperature ikke er tilgængelig i MiniPanelet
     DataStringtoSD += ",";  // Komma først
     for (int i = 0; i < 3; i++)    // read 2 parameters and append to the string:
      {
        float datavalue = parameter[i];
        DataStringtoSD += String(datavalue);
        if (i < 3) 
          DataStringtoSD += ","; 
      } 

     DataStringtoSD=DataStringtoSD+drawString; //drawString er af typen Byte. Blir også tilføjet strengen
     DataStringtoSD=DataStringtoSD+","; 

//      DataStringtoSD = ",12.33,4.567,21.99,10,"; //len = 22
//      
//      // så er der forskel på afsendte data, alt afhængig af om sekundet er lige eller ulige.
//      if (second(t) % 2 == 0)
//        DataStringtoSD = ",12.00,1.00,21.99,10,"; //len = 21
//      else
//        DataStringtoSD = ",17.50,0.00,22.12,12,"; //len = 21
      
      
   } // if (crc==1309)

 
  DataStringtoSD=DataStringtoSD+crc;
  DataStringtoSD=DataStringtoSD+","; // husk at afslutte med ,

  Serial.print("Her kommer DataStringtoSD strengen som indeholder crc : ");   
  Serial.println(DataStringtoSD);
  Serial.print("String Length er : ");   
  Serial.println(DataStringtoSD.length()); // og så lige længden på strengen
  
    
  // Konvertering af string til char
  len = DataStringtoSD.length()+1;
  DataStringtoSD.toCharArray(SendPayload, len); //len=22+1 for volt,amp temp
//*******************************************************************
// Radio delen af NRF24L01_ReceiveFromDisplay 1= dato 2=motor mm
//*******************************************************************
  if (len <6) return;  //hvis der ikke er data i payload så hop ud. 11/5-2018
  while ( !ACKdone ) 
   {
      // First, stop listening so we can talk
      radio.stopListening();
      radio.openWritingPipe(pipes[1]);    //Så kan der sendes TX 15-01-2015
      radio.openReadingPipe(0,pipes[0]);  //Så kan der sendes TX 15-01-2015    

// delay(20); //test 9/8

      // Send the last CRC back 
      radio.write(SendPayload,len);
      Serial.print("Sent response. "); Serial.println(SendPayload);   //payload mangler nogel gange ?

       //ikke med i den originale kode 
      if (ExitTimer >10)
        {
         Serial.println("Sent response failing. Return to main program "); 
         ACKdone==true;
         delay(2000);
        }
      else
        ExitTimer++;
      Serial.print("ExitTimer = "); Serial.println(ExitTimer);  // ExitTimer = 1 er fino
      


      RecvPayload[0] = 0;  // Clear the buffers      
      SendPayload[0] = 0;  // Clear the buffers
      
      // Now, resume listening so we catch the next packets.
      radio.openWritingPipe(pipes[0]);    //Samme som i setup RX 15-01-2015
      radio.openReadingPipe(0,pipes[1]);  //Samme som i setup RX 15-01-2015
      radio.startListening();


    // Wait here until we get a response, or timeout
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 500 )
        timeout = true;

    // Describe the results
     if ( timeout )
       {
         printf("Failed, response timed out.\n\r");
//         StopMotorEW(); //ellers stopper motoren ikke hvis data går i hegnet
         //StopMotorUD(); //ellers stopper motoren ikke hvis data går i hegnet  
         Error=Error+1;
         if (Error >2) return;
      }
    else
     {
      // Hent response, sammenlign det i en case
      len = radio.getDynamicPayloadSize();
      radio.read( RecvPayload, len );
      RecvPayload[len] = 0;

      // hvad kom der ind
//      printf("Got response size=%i value=%s\n\r",len,RecvPayload);
//      Serial.print("R:");
//      Serial.print(RecvPayload);
//      Serial.println();
 
      val = strtok (RecvPayload,","); // Hiver variable ud af stringen (som char)
      okval= atoi(val);               // laver char om til en integer

//      Serial.print("okval= ");
//      Serial.println(okval);

        
      // check at det er det korrekte check ciffer der hører til den rigtige string
   switch (crc)
       {
    case 1280: if (okval==333) // sender kun Lysværdi tilbage ...som ER gjort. Bruges kun til Stort Dispaly
         {
           ACKdone=true; 
           // // RadioLink_is_UP=true;
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: //ButtonMotoUDOnState = atoi(val);  
               break;
               case 2: ButtonMotorMoveEastState = atoi(val);               
               break;
               case 3: waitMinutesToNextTrackUpdate = atoi(val);               
               break;
               case 4: //VoltAmpSampleRate = atoi(val);               
               break;
               case 5: //SensorThreshold = atoi(val);               
               break;
               case 6: SetdifferenceEastWest = atoi(val);               
               break;
               case 7: //SetdifferenceUpDown = atoi(val);               
               break;
              }
             } //while
            count=0;
            //digitalWrite(38, LOW);
            //Serial.println("Data modtaget fra Diaplay ACK 333 (SKRIVER DATA SetMotor)");
            //delay(8);  //Delay i stedet for serial.print

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

         }//if (okval==333)
    break;         

    case 1281: if (okval==444) // sender kun Lysværdi tilbage ...som ER gjort. Bruges kun til MiniRemoten
         {
           int Tempval   = 0;
           ACKdone=true; 
           // RadioLink_is_UP=true;
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: //ButtonMotoUDOnState = atoi(val);  
               break;
               case 2: ButtonMotorMoveEastState = atoi(val);               
               break;
               case 3: Tempval = atoi(val);  //læses ikke ind             
               break;
               case 4: Tempval = atoi(val);  //læses ikke ind              
               break;
               case 5: Tempval = atoi(val);  //læses ikke ind               
               break;
               case 6: Tempval = atoi(val);  //læses ikke ind              
               break;
               case 7: Tempval = atoi(val);  //læses ikke ind               
               break;
              }
             } //while
            count=0;
 //           digitalWrite(2, HIGH);
 //           digitalWrite(38, LOW);
 //           Serial.println("Data modtaget fra Diaplay ACK 444 (SKRIVER DATA SetMotor)");

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

         }//if (okval==444)
    break;         

    case 1309: if (okval==222)   // Sætter motor parameterne og sample værdier.  Bruges kun til Stort Display
          { 
           ACKdone=true;  
           // RadioLink_is_UP=true;
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: //ButtonMotoUDOnState = atoi(val);  
               break;
               case 2: ButtonMotorMoveEastState = atoi(val);               
               break;
               case 3: waitMinutesToNextTrackUpdate = atoi(val);               
               break;
               case 4: //VoltAmpSampleRate = atoi(val);               
               break;
               case 5: //SensorThreshold = atoi(val);               
               break;
               case 6: SetdifferenceEastWest = atoi(val);               
               break;
               case 7: //SetdifferenceUpDown = atoi(val);               
               break;

              }
             } //while
            count=0;
  //          digitalWrite(2, HIGH);
  //          digitalWrite(38, LOW);
 //           Serial.println("Data modtaget fra Diaplay ACK 222 (SKRIVER DATA SetMotor)");

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

          } // if (okval==222)
    break;


    case 1310: if (okval==555)   // Sætter motor parameterne.  Bruges kun til MiniRemoten
          { 
           int Tempval   = 0;
           ACKdone=true;   
           // RadioLink_is_UP=true;
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: //ButtonMotoUDOnState = atoi(val);  
               break;
               case 2: ButtonMotorMoveEastState = atoi(val);               
               break;
               case 3: Tempval = atoi(val);  //læses ikke ind                
               break;
               case 4: Tempval = atoi(val);  //læses ikke ind       
               break;
               case 5: Tempval = atoi(val);  //læses ikke ind      
               break;
               case 6: Tempval = atoi(val);  //læses ikke ind            
               break;
               case 7: Tempval = atoi(val);  //læses ikke ind 
               break;

              }
             } //while
            count=0;
    //        digitalWrite(2, HIGH);
    //        digitalWrite(38, LOW);
    //        Serial.println("Data modtaget fra Diaplay ACK 555 (SKRIVER DATA SetMotor)");

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

          } // if (okval==555)
    break;



    case 1966: 
         if (okval==111)   // hvis ok er data valide og jeg behøver ikke gensende datoen. Bruges kun til Stort Display
          { 
           ACKdone=true; 
           // RadioLink_is_UP=true;
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           timer = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            {
             ++count;
             switch (count)
              { 
               case 1: minutter = atoi(val);
               break;
               case 2: sekunder = atoi(val);
               break;
               case 3: dag = atoi(val);
               break;
               case 4: maaned = atoi(val);
               break;
               case 5: aar = atoi(val);
               break;
              }
             } //while
            count=0;
   //         digitalWrite(2, HIGH);
   //         digitalWrite(38, LOW);
   //         Serial.println("Data modtaget fra Diaplay ACK 111 (SKRIVER DATA SetTime)");
          
        } //  if (okval==111)
    break;
         } // switch (crc)

    if (!ACKdone) // Ved ikke om denne skal slettet igen ??
        { 
//         StopMotorEW(); //ellers stopper motoren ikke hvis data går i hegnet
         //StopMotorUD(); //ellers stopper motoren ikke hvis data går i hegnet  
        }
        //Serial.println("Buffer Reset"); 
        DataPayload[len] = 0;  // Put a zero at the end for easy printing
        RecvPayload[0]   = 0;  // Clear the buffers          
        DataPayload[0]   = 0;  // Clear the buffers  
      }//else
    }//  while ( !ACKdone )

  RecvPayload[0] = 0;  // Clear the buffers
  DataPayload[0] = 0;  // Clear the buffers  
//  Serial.println("-------------------Finish med at hente og sende.  Klar til ny loop.---------------------");         
//  Serial.println();       
  // læser digitale værdier fra kontakterne til manuel motorstyring
 
  NoLink=0; 
  ReadButtons();                 
  
}

// ************************************
// Get the contacts value on/off 
// and move the motor or whatever
// ContactMotorEWOn=46;      // fjeder kontakt nr 2 fra højre on=46. (45 gruges som +5v og sættes HIGH)
// ContactMotorMoveEast=43;  // 3 vejskontakt  ned=west(41)  midten(42)  op=EAST(43)
// ContactMotorMoveWest=41;  // 3 vejskontakt  ned=west(41)  midten(42)  op=EAST(43)
// ************************************
void ReadButtons()
{
//  Serial.println();
//  Serial.println("--------Start ReadButtons------------- ");  //hvis ikke denne er her, vil uret ikke vises   
//  Read the contacts

//  ButtonMotorMoveEastState = digitalRead(ButtonMotorMoveEast); // kontakt op
//  ButtonMotorMoveWestState = digitalRead(ButtonMotorMoveWest); // kontakt ned
 
//  motor1 run E/W, motor2 run U/D, kontakt U/D-E/W, NextTracktime, VoltAmpSampleRate, Sensorthreshold, SetdifferenceEastWest, SetdifferenceUpDown
//  parameterbool[3]={ButtonMotoEWOnState,ButtonMotoEWOnState,ButtonMotorMoveEastState}; 

  //Serial.print("Motor = "); Serial.println(ButtonMotoEWOnState);
  if ( ButtonMotoEWOnState == 0) 
    digitalWrite(2, HIGH);  
  else
    {
//       StopMotorEW(); //ellers stopper motoren ikke
  //     Serial.println("MotorEWOn OFF ");
       digitalWrite(2, LOW);
     }
  //  if (ButtonMotoUDOnState ==1) StopMotorUD(); //ellers stopper motoren ikke   
  //   {
  //     StopMotorEW(); //ellers stopper motoren ikke
  //     Serial.println("MotorUDOn OFF ");
  //   }

    
    // Først East West kontakten
    if ( ButtonMotoEWOnState == 0)  // hvis fjederkontakten er trykket ned så gør følgende
     {
//       Serial.println("--------MotorEWOn Kontakten er aktiv ------------- ");
     
        switch (ButtonMotorMoveEastState) //Was the switch set to East ?
         {
         case 0:
           //do something when var equals 1
            Serial.println("Manuel, Going East");
//            goToEastManuel();
           break;
         }

        switch (ButtonMotorMoveWestState) //Was the switch set to West ?
         {
         case 0:
           //do something when var equals 1
            Serial.println("Manuel, Going West");
//            goToWestManuel();
           break;
         }
     }

//Serial.println("--------Stop ReadButtons------------- ");    
}




//**********************************
// Read Serial input
//**********************************
void ReadSerialInput() //Husk at terminal vindue skal være med "No Line ending"
{

  time_t t = now();



   // ***** Hvis en tast 1-4 sendes via seriel via kabel, vil den gå i manuel mode  *****
   if (Serial.available() > 0)    // for incoming serial data
   {
     incomingByte = Serial.read();

     // hvis der sendes et 1 tal: GoEast
     if (incomingByte == 49) 
     {
       Serial.println("Go East. Husk at trykke 3 for at stoppe igen, end kontakten virker ikke ");
       delay(1000);
//       goToEast();
     }
     // hvis der sendes et 2 tal: GoWest
     if (incomingByte == 50)  
     {
       Serial.println("Go West. Husk at trykke 3 for at stoppe igen, end kontakten virker ikke");
       delay(1000);       
//       goToWest();
     }

    // hvis der sendes et r  resætter hele systemet. Kun til test formål  26/5-2018
    if (incomingByte == 114)  
     {
       Serial.println("RESET af system.  Hiv kablet ud nu!   Genstarter om 5 sek.");
       delay(3000);       
       tone(BeepPin, 3000, 500);   // pin, hz, duration  Melder klar
       delay(1000);
       tone(BeepPin, 3000, 500);   // pin, hz, duration  Melder klar
       delay(1000);
       tone(BeepPin, 3000, 500);   // pin, hz, duration  Melder klar
       delay(1000);
       tone(BeepPin, 3000, 500);   // pin, hz, duration  Melder klar
       delay(2000);

       incomingByte = 53;    //Forsæt readsensor
       while(1){}            // vi lader watchdog'en bide efter 4 sekunder
     }     


     // Bliver kun vist hvis der er incomming serial data
     Serial.print("incomingByte Modtaget: ");
     Serial.println(incomingByte,DEC);
     delay(2000);
   } //*****  Read serial end ***

}

//**********************************
// The loop where everything happens
//**********************************
void loop(void) 
{
  wdt_reset();           // Watchdog reset  

  // Starter op i alm læs sensor mode. incomingByte er default =53.
  ReadSerialInput();

  
  //For test purpose 
  //Serial.print("Main loop ");

  //delay(2000);
 
  // modtager data fra Displayet
  NRF24L01_ReceiveCommandFromDisplay();                        // Læser East/West fra remoten. Motoren blir styret af remoten. 


//delay(10);                                              // Sænker hastigheder fra 46 omg til 32 omg i ReadLightSensorEastWest
}

