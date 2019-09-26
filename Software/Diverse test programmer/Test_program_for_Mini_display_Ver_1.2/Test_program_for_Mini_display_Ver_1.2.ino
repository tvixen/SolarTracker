//*************************************************************************
// Version: Arduino IDE 1.6.9 (7/7-2016)
//
// Author: Tim Milgart
//
// Start Date: 13.01.2015
// License: Tim Milgart
// Update: 10-07-2016
// Update 14/07-2016 på job 
// Denne version Test_program_for_Mini_display_taken_from_Display_2.2.3-_Ver_1.1 ændres til Test_program_for_Mini_display_Ver_1.1
// Test_program_for_Mini_display_Ver_1.1 passer til Soltracker_til_MiniPanel_RF_ver_1.1
// Update 14/07-2016
// Soltracker_til_MiniPanel_RF_ver_1.2




// Time lib
#include "Time.h"
// RTC lib
#include "DS1302RTC.h"
// Show memory lib
#include "MemoryFree.h"
#include <SPI.h>
//NRF24L01+
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"



//Define solartracker ports, valiables and static
//Digital ports
// 0 , 1 reserved for serial com
const byte MotorUDOn           = 3;  // relæ motor2 on/off til op/ned
const byte MotorMoveUpDown     = 3;  // relæ motor2 styring op/ned af panelet
const byte MotorMoveEastWest   = 3;  // relæ motor1 øst/vest styring af rammen
const byte MotorEWOn           = 3;  // relæ motor1 on/off til øst/vest 


//Buttons and Potmeters
const byte ButtonMotorEWOn     = 46;  // fjeder kontakt nr 2 fra højre on=46. (45 gruges som +5v og sættes HIGH)
const byte ButtonMotorMoveEast = 43;  // 3 vejskontakt  ned=west(41)  midten(42)  op=EAST(43)
const byte ButtonMotorMoveWest = 41;  // 3 vejskontakt  ned=west(41)  midten(42)  op=EAST(43)
const byte ButtonMotorUDOn     = 48;  // fjeder kontakt nr 3 fra højre on=48. (47 gruges som +5v og sættes HIGH) Kan skifte f.eks display ml. 1 og 2. Eller anden funktion


//Global incomming Analog ports (DB9 stik 10,11,12,13,14,15)
//int solarCurrentSensor       = 0;  // modatget fra Panel på A10 Orange kabel, Lille blå print 
int lightSensorEastUpValue     = 0;  // modatget fra Panel på A11 Blå kabel                     
int lightSensorWestUpValue     = 0;  // modatget fra Panel på A12 Grøn kabel                    
int lightSensorEastDownValue   = 0;  // modatget fra Panel på A13 Blå+hvid kabel                
int lightSensorWestDownValue   = 0;  // modatget fra Panel på A14 Blå+hvid kabel                
//int solarTempSensor          = 0;  // modatget fra Panel på A15 Brun og hvid kabel            
// +5V Hvid+Orange   
// GND Brun



//Global Sensor Variables
float sample      = 0.0;
float Voltage     = 0.0;
float Amps        = 0.0;
float Temperature = 0.0; 
int mappedVoltValue = 0;
float totamps     = 0.0; 
float avgamps     = 0.0;
float amphr       = 0.0;
float TotalProducedEnergyHours = 0.0;   // Energi
float Energysekunder           = 0.0;
float TotalProducedEnergysekunder = 0.0;
float ampsec       = 0.0;



//Global Date Variables
byte BadRTC     = 0;  // Check if RTC i working or not working =0
int startyear   = 0;
int startmonth  = 0;
int startday    = 0;
int starthour   = 0;
int startminute = 0;
int startsecond = 0;
unsigned long currentmillis=0;  // gemmer antal millisekunder. range from 0 to 4,294,967,295
int viser       = 0;    // used in a meter to show sun/volt
int ProgSeconds = 0;    // sekunder der bruges til at programmere med, nedtælling
long LoopCount  = 0;    // bruges til energi beregning, hvor mange gange har der været loop.
int NextTrackUpdate = 0;     // Minutes to nexttrackupdate realtime. This value is comming from Panel.
boolean SetNextTrackinSeconds = true;
int incomingByte                                    = 53;  // Serial read, til at styre E/W motor manuelt  53=ascii 5


// ServiceMenu parametre
int Menuvalg           = 0;
byte servicemenu       = 0;             //0= alm menu 1= service menu
int SensorThreshold    = 0;             //Bruges til at lukke ned for følsomheden på solsensor, men bruges ikke mere 
int TempValue          = 0;
int VoltAmpSampleRate  = 60;
int SleepTime          = 1;
int waitMinutesToNextTrackUpdate = 1;  //This user value is in minutes. 1 equal one minut 5 eual 5 minuttes. Default is 2. 
int WifiOn             = 0;            //Wifi On=1 Off=0. Bruger integer da den ellers ikke vil skrives ud i service menuen.
int SetdifferenceEastWest = 30;        //Tolerancen på hvornår panelet skal dreje (hvis sensorværdien er mere end 40 i forskel fra East til West)
int SetdifferenceUpDown   = 130;       //Tolerancen på hvornår panelet skal vippe (hvis sensorværdien er mere end 10 i forskel fra up til down)
int GraphTime             = 10;        // This user value is in seconds. Sets the time of live graph draw from the panel in Servicemenu2
char AmpLine[232]         = "";        // Set of values to show a graph for Amps
char VoltLine[232]        = "";        // Set of values to show a graph for Volts
int  LCDcount             = 0;         // Global counter for graph in servicemenu2
String DataStringtoLCD    = "";        // Part of a graph in servicemenu2

//Global Bolean 
bool ButtonMotoEWOnState =1;           //1=Motor OFF  0=Run
bool ButtonMotoUDOnState =0;
bool ButtonMotorMoveEastState =1;      //1=Motor West
bool ButtonMotorMoveWestState =0;
bool East = HIGH; //Active high
bool West = LOW;  //Active Higi
bool Down = HIGH; //Active High
bool Up   = LOW;  //Active High
bool Run  = HIGH; //Active High
bool Stop = LOW;  //Active LOW



// Define pin RTC1302 
// DS1302:         CE pin    -> Arduino Digital 2  RST  MEGA 39
// DS1302:         I/O pin   -> Arduino Digital 3  DAT  MEGA 38
// DS1302:         SCLK pin  -> Arduino Digital 4  CLK  MEGA 40
// DS1302:         Remember 5V and not 3,3V
// Init the DS1302 
// Set pins:  RST, DAT,CLK
DS1302RTC RTC(39, 38, 40);



// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 kan godt ændres til andre pins. (CE, CSN)
// Set Power Amplifier (PA) level to one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
// The power levels correspond to the following output levels respectively: NRF24L01: -18dBm, -12dBm,-6dBM, and 0dBm
//PRINT LAYOUT for Mega:
//1= GND      2=VCC
//3=CE(45)    4=CSN(49)
//5=CSK(52)   6=MOSI(51)
//7=MISO(50)  8=IRQ(nc)
RF24 radio(45,49);  //(CE,CSN)
const uint64_t pipes[2] = { 0xDEDEDEDEE1LL, 0xDEDEDEDEE2LL };  //WritingPipe, ReadingPipe
boolean stringComplete = false;  // whether the string is complete
boolean RadioLink      = false;  // is there a radiolink
char *val;
int count = 0;
char SendPayload[32] = "";       // Empty payload for start
char RecvPayload[32] = "";
int crc=0;


// For use in loop
long counter_for_numbers_of_loops = 0; //Number of loops
int  prevsec                      = 0; //Previous seconds
int Error                         = 0; //RF Errors
long TotalError                   = 0; //Total number of errors 


//********************
//Initialize and setup
//********************
void setup(void) 
{
 //Initialize ports and variables  
  analogReference(DEFAULT); //INTERNAL1V1 = 1.1V INTERNAL2V56 = 2.56V , DEFAULT = 5 volt
  pinMode(53,                        OUTPUT);  // SPI CS bruges ikke, men skal være sat til OUTPUT 
  pinMode(ButtonMotorUDOn,     INPUT_PULLUP);  // fjeder kontakt nr 3 fra højre on=47. skal give +5v før det virker. 
  pinMode(ButtonMotorEWOn,     INPUT_PULLUP);  // fjeder kontakt nr 2 fra højre on=46. 
  pinMode(ButtonMotorMoveEast, INPUT_PULLUP);  // 3 vejskontakt  ned=west(41)  midten(stel)  op=EAST(43)  
  pinMode(ButtonMotorMoveWest, INPUT_PULLUP);  // 3 vejskontakt  ned=west(41)  midten(stel)  op=EAST(43)  
  pinMode(MotorEWOn,                 OUTPUT);  // Motor1 on-off
  pinMode(MotorMoveEastWest,         OUTPUT);  // East 0, West 1
  pinMode(MotorUDOn,                 OUTPUT);  // Motor2 on-off
  pinMode(MotorMoveUpDown,           OUTPUT);  // Up 0,  Down 1

  //Get the whole string from RTC      
  tmElements_t tm;   
  
  
  //  initialize serial communication at 57600 bits per second:
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
    startyear =year(t);
    startmonth= month(t);
    startday= day(t);
    starthour =hour(t);
    startminute= minute(t);
    startsecond= second(t);
    minute(t);  // returns the minute for the given time t 

    
    Serial.println("Version: SolTracker_til_Display_ver 2.2.3");

    //Set watch
    time_t myTime;
    myTime = RTC.get();


    // init value for calc- men er ikke sikker på det er nødvendigt
    Voltage             = 0.0;
    Amps                = 0.0;
    Temperature         = 0.0; 
    sample              = 0.0; 
    mappedVoltValue     = 0;

    //  Disse bruges til gennemsnitsberegning 
    totamps = 0.0; 
    avgamps = 0.0;
    amphr   = 0.0;
    TotalProducedEnergyHours = 0.0; 
    ampsec = 0.0;

  

  //NRF24L01+ begynder init her    
  printf_begin();

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(125);
  
  radio.enableDynamicPayloads();
  radio.setRetries(5,15);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableAckPayload();

  radio.openWritingPipe(pipes[0]);   // Open different pipes when writing. Write on pipe 0, address 0
  radio.openReadingPipe(0,pipes[1]);  
  
  radio.startListening();
  radio.printDetails();

  Serial.println("SETUP DONE !");

  // Så meget fri RAM er der tilbage
  Serial.print("Free RAM: ");  Serial.println( freeMemory() );

}

// *************CRC Check******************************
// Funktion til hive de sidste check cifre ud af strengen
// og derefter konverterer det til en integer
// ****************************************************
void CRC_Check(int len)
{
  char tmpcrc[5] = ""; 

  for (int i = 0; i < 4; i++) 
    tmpcrc[i]=RecvPayload[len-6+i];
  val = strtok (tmpcrc,",");
  crc = atoi(val);
}

// ***************DISPLAY************************************************
// Send procedure. Med parameter(int data)
// 1=dato og 2= motor1 on/off, motor2 on/off, drawString, NextTracktime, VoltAmpSampleRate, Sensortreshold
//  ,0,0,0,20,99,99,1309, = 21 i alt  
//   1, 2, EW, DS, TT, SR, ST,CRC
//***********************************************************************
void NRF24L01_SendToPanel(byte DataMode)
{
  int okval=0;
  bool ACKdone = false;
  int countup=0;
  char DateTime[27] ="";  // Den nye metode
  char DataPayload[32] = "";
   
  String DataStringtoSD = "";  
  byte NextCaseDataMode=0;



 //Serial.println(" ------------ ReadButtons --------------------");  
   //Read the contacts
   
   //ButtonMotoEWOnState          = 1; //Fjederkontakt   1=Stop
     ButtonMotoUDOnState          = 1; //Fjederkontakt   1=Stop
   //ButtonMotorMoveEastState     = 0; // kontakt op     1=move to 
   //ButtonMotorMoveWestState     = 4; // kontakt ned / sendes ikke med i streng
     waitMinutesToNextTrackUpdate = 4; // kontakt ned
     VoltAmpSampleRate            = 5;
     SensorThreshold              = 0;
     SetdifferenceEastWest        = 7;
     SetdifferenceUpDown          = 8;

     
   
//  motor1 run E/W, motor2 run U/D, kontakt U/D-E/W, NextTracktime, VoltAmpSampleRate, Sensorthreshold, SetdifferenceEastWest, SetdifferenceUpDown
//       0,0,1 = run,run,west

  switch (DataMode)
    {
       case 3:{//  Sender motor1 run E/W, motor2 run U/D, kontakt U/D-E/W, NextTracktime, VoltAmpSampleRate, Sensortreshold og får LightSensorValues tilbage.
              bool parameterbool[3]={ButtonMotoEWOnState,ButtonMotoUDOnState,ButtonMotorMoveEastState,}; // 3 værdier af sammt type (boolean) pakkes til een string
              DataStringtoSD += ",";  // Komma først
              for (int i = 0; i < 3; i++)    // read 2 parameters and append to the string:
               {
                 bool datavalue = parameterbool[i];
                 DataStringtoSD += String(datavalue);
                 if (i < 3) 
                   DataStringtoSD += ","; 
               } 

              int parameter[5]={waitMinutesToNextTrackUpdate,VoltAmpSampleRate,SensorThreshold,SetdifferenceEastWest,SetdifferenceUpDown}; // 5 værdier af sammt type (integer) pakkes til een string
              for (int i = 0; i < 5; i++)    // read 2 parameters and append to the string:
               {
                 int datavalue = parameter[i];
                 DataStringtoSD += String(datavalue);
                 if (i < 5) 
                 DataStringtoSD += ","; 
               } 
              DataStringtoSD=DataStringtoSD+1280;
              DataStringtoSD=DataStringtoSD+","; // husk at afslutte med ,
              NextCaseDataMode=DataMode;
             } 
      break;
    }

 // Serial.print(" Outgoing "); Serial.println(DataStringtoSD);  // Bare så jeg lige kan kontrollere hvad der er i strengen default er den = (,1,1,1,2,60,0,1309,) 
                                                               // ButtonMotorMoveEastState blir kun 0 når kontakten vippes op

  
  uint8_t  len = DataStringtoSD.length()+1;
  DataStringtoSD.toCharArray(SendPayload, len);
//***************************
// Radio delen af send 1= dato 2=motor mm
//***************************

   while ( !ACKdone ) 
     {
       // First, stop listening so we can talk
       radio.stopListening(); // stop med at lytte og send vores streng
       radio.openWritingPipe(pipes[1]);    //Så kan der sendes TX 15-01-2015
       radio.openReadingPipe(0,pipes[0]);  //Så kan der sendes TX 15-01-2015


        radio.write(SendPayload,strlen(SendPayload)+1); //1 Dato strengen,16,30,59,01,01,2015,1966, eller 2 ,1,0,1,20,99,99,1309,
 //       Serial.println(SendPayload); // og så lige længden på strengen  
 
      // Genoptag lytning for næste packet, som skal være den sidste del af strengen der sendes tilbage fra modtageren også kaldet ACK
      radio.openWritingPipe(pipes[0]);    //Samme som i setup RX 15-01-2015
      radio.openReadingPipe(0,pipes[1]);  //Samme som i setup RX 15-01-2015
      radio.startListening();

//      Serial.println("Waiting for ACK ");
    
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
         Error=Error+1;
         // Alle værdier resættes, så de viser 0 hvis radiolink mistes. Så beregnes der ikke på forkerte tal.
         Voltage=0.0;
         Amps=0.0;
         Temperature=0.0; 
         lightSensorEastUpValue   = 0;
         lightSensorWestUpValue   = 0;
         lightSensorEastDownValue = 0;
         lightSensorWestDownValue = 0;
         if (Error >5) return;
       }
      else
       {
        // Hent response, sammenlign det i en case
        len = radio.getDynamicPayloadSize();
        radio.read( RecvPayload, len );
        RecvPayload[len] = 0;

        // hvad kom der ind   ***denne skal blive
        for (int i = 0; i < 32; i++) 
          DataPayload[i]=RecvPayload[i];

 //Serial.print("incomming "); Serial.println(DataPayload);
       
        CRC_Check(len); //Kald crc check med længden på RecvPayload

        okval=crc;
        // Delay da den ellers bliver kvalt i hastighed efter fjernelsen af serial.print...desværre. default=(60)
        delay(10);

          
        // check at det er det korrekte check ciffer der hører til den rigtige string
        switch (NextCaseDataMode)
         {

           case 3: if (okval==1280)   // hvis ok er data valide. Uden dato, lysværdi retur
                   {             
                      ACKdone=true; 
                     // Serial.println("ACK OK from CRC, Display sending 333 to confirm");
                      radio.openWritingPipe(pipes[1]);    //Så kan der sendes TX 15-01-2015
                      radio.openReadingPipe(0,pipes[0]);  //Så kan der sendes TX 15-01-2015
                      radio.stopListening();
                      radio.write("333", 3);
                      radio.openWritingPipe(pipes[0]);    //Samme som i setup RX 15-01-2015
                      radio.openReadingPipe(0,pipes[1]);  //Samme som i setup RX 15-01-2015
                      radio.startListening();
                     // digitalWrite(12, LOW);  
                     
                      val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
                      lightSensorEastUpValue= atof(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
                      while ((val = strtok (NULL, ",")) != NULL)
                       {
                        ++countup;
                        switch (countup)
                         {
                          case 1: lightSensorWestUpValue = atoi(val);  
                          // test af graph. Her skal den putte værdien af taller ind i et array, når arrayet er fyldt op skal den første skiftes ud med den anden osv
                          
                          break;
                          case 2: lightSensorEastDownValue = atoi(val);               
                          break;
                          case 3: lightSensorWestDownValue = atoi(val);               
                          break;
                          case 4: NextTrackUpdate = atoi(val);
                         } //Switch
                       } //while
                   } // if (okval==1280)
           break;
         }// Switch
        

        if (!ACKdone) // Ved ikke om denne skal slettet igen ??
        { 
          //Serial.println("CRC IKKE FUNDET ..... RETURN");       
            // Alle værdier resættes, så de viser 0 hvis radiolink mistes. Så beregnes der ikke på forkerte tal.
            Voltage=0.0;
            Amps=0.0;
            Temperature=0.0; 
            lightSensorEastUpValue   = 0;
            lightSensorWestUpValue   = 0;
            lightSensorEastDownValue = 0;
            lightSensorWestDownValue = 0;
            return; // skal sikre at der ikke modtages så meget crap og at den ikke hænger når det sker
        }


        
         if (Error > 5)                // Ingen ACK så vi timerout efter 5 omgange
          {
            //Serial.println("Time OUT");       
            ACKdone=true;
          }      
          
          RecvPayload[0] = 0;  // Clear the buffers  
          ++count;
      } //else
     }  //while
   SendPayload[0] = 0;
   radio.openWritingPipe(pipes[0]);    //Samme som i setup RX 20-01-2015
   radio.openReadingPipe(0,pipes[1]);  //Samme som i setup RX 20-01-2015
   radio.startListening(); 
}
// ************************************
// Skriver i display hvis der er fejl i RTC ved opstart
// ************************************
void RTCerror() 
{
  Serial.println(" RTC Error ");
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
//   Get the contacts value on/off 
//   For test purpose we add read of serial port

 // Starter op i alm læs sensor mode
  
   // Hvis en tast 1-4 sendes via seriel via kabel, vil den gå i manuel mode
   if (Serial.available() > 0)    // for incoming serial data
   {
     incomingByte = Serial.read();
     if (incomingByte == 49) // hvis der sendes et 1 tal: GoEast
     {
//       SensorThreshold = 1;
       ButtonMotoEWOnState = 0;
       ButtonMotorMoveEastState= 0;
       Serial.println("Husk at trykke 3 for at stoppe igen ");
     }
    if (incomingByte == 50)  // hvis der sendes et 2 tal: GoWest
     {
//       SensorThreshold = 2;
       ButtonMotoEWOnState = 0;
       ButtonMotorMoveEastState= 1;
       Serial.println("Husk at trykke 3 for at stoppe igen ");
     }
    if (incomingByte == 51)  // hvis der sendes et 3 tal: Stop
     {
//       SensorThreshold = 3;
       ButtonMotoEWOnState = 1;
       ButtonMotorMoveEastState= 0;
       Serial.println("Motor er stoppet ");
     }
    if (incomingByte == 52)  // hvis der sendes et 4 tal: Update=1
     {
//       SensorThreshold = 4;
       Serial.println("waitMinutesToNextTrackUpdate = 1");              
     }
     if (incomingByte == 53)  // hvis der sendes et 5 tal: Normal
     {
//       SensorThreshold = 5;
       ButtonMotoEWOnState = 1;
       ButtonMotorMoveEastState= 1;
     }    
    if (incomingByte == 54)  // hvis der sendes et 6 tal: Update=2
     {
//       SensorThreshold = 6;
       Serial.println("waitMinutesToNextTrackUpdate = 2");
     }
    if (incomingByte == 48)   // hvis der sendes et 0 tal sættes tiden Ascii=48
     {
//       SensorThreshold = 0;   //Sætter tid og dato i den anden ende, til det den er hardcoded til
       Serial.println(" ");
       Serial.print("Tiden er sat :");
       time_t t = now();      //Hent tiden
       Serial.print(hour(t));   Serial.print(":");       //Printer dette prints tid ud og ikke remoten...det er kun for sjov
       Serial.print(minute(t)); Serial.print(":");        
       Serial.println(second(t)); 
     }
     
     Serial.print("Modtaget: ");
     Serial.println(incomingByte,DEC);
   }

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
  currentmillis = millis();     // calculate time in milisec
  long  EnergyTime = currentmillis/1000; // convert millisec to sec som bruges i kwh beregningen.
//////////////////////////////////////////////// current and energy calculation //////////////////////
  time_t t = now(); // henter tiden.

  if (((second(t) % 1 == 0) && (prevsec != second(t))))  //hvert sekund ..Det hele beregnes 1 gang i sekundet. No more, No less.
   {  
     // Kontrollere lige om tiden er korrekt. så den her kommer hvert sekund.
     Serial.print("prevsec= ");  Serial.println(prevsec);       
     Serial.print("seconds= ");  Serial.println(second(t)); 
     Serial.println(" ");    Serial.println(" ");
     Serial.print(hour(t));  Serial.print(":");       
     Serial.print(minute(t));Serial.print(":");        
     Serial.println(second(t)); 

     prevsec=(second(t));  

     LoopCount = LoopCount +1;                   // counter for calculate
     
   }   
//////////////////////////////////////////////// current and energy calculation //////////////////////

    Serial.println(" ");    Serial.println(" ");
    Serial.print(hour(t));  Serial.print(":");       
    Serial.print(minute(t));Serial.print(":");        
    Serial.print(second(t)); 

    Serial.print("       Errors = "); Serial.print(Error);  
    Serial.print("       TotalErrors = ");Serial.print(TotalError);          

    TotalError+=Error;
    //Reset Error igen, så den er klar til næste fejl
    Error=0;
    
    Serial.print("       Antal loop gange "); Serial.print(counter_for_numbers_of_loops);   
    counter_for_numbers_of_loops++;
    
    // Så meget fri RAM er der tilbage
    Serial.print("       Free RAM: ");  Serial.println( freeMemory() );
    Serial.println(" ");    Serial.println(" ");
    
    prevsec=(second(t)); 

    // læser digitale værdier fra kontakterne til manuel motorstyring
    ReadButtons();                 


   // Sender data til modtager        
   NRF24L01_SendToPanel(3);  // 3= set motor og samplerate (crc=1280)og Lysværdi retur   



  //Radio lukkes ned
  //radio.powerDown(); 

  delay(10);  
  
  //Radio startes op
  //radio.powerUp();
  
}
