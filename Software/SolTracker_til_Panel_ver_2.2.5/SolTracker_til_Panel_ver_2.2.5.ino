//*************************************************************************
// Version: Arduino IDE 1.6.4
//
// Author: Tim Milgart
//
// Start Date: 10.10.2014
// License: Tim Milgart
// Update: 05-02-2015
// Virker med modsvarende version af Panel
// Har puttet alt koden fra RNF24L01_PANEL_ver_2.0.8 over i denne version SolTracker_til_Panel_ver_2.1.0
// Har lavet ny kode til ver 2.1.0 der kommer fra RNF24L01_Panel_ErrorCode_ver.1.0.1 NRF24L01_ReceiveFromDisplay()
// Når Display sender data til Panel, svarer Panel med nye data men med samme crc
// På den måde får jeg en hurtig response og behøver ikke vende transmissionen
// Display sender SendToPanel(2) " ,1,0,1,20,99,99,1309, " som er motor, og får ",12.33,45.66,78.99,101,1309," tilbage
// Som er Volt, Amp, Temp og DrawString.
// Display sender SendToPanel(1) " ,16,30,59,01,01,2015,1966, " som er dato og tid, og får "999,999,999,999,1966," tilbage
// som er LightSensor Values.
// Display sender SendToPanel(3) " ,,11,12,13,14,15,16,1280, " som er uden dato og tid, og får "999,999,999,999,1966," tilbage
// som er LightSensor Values. Så sendes dato&tid kun en gang i døgnet.
// Savet som version 2.1.4, så Display og Panel matcher i versionsnummer.
// Sat dioder på 32,38,26 så jeg kan se om kontakterne virker. 28.124 Bytes
// Ver 2.1.5 fjernet LUE LUW LDE LDW og byttet lidt om på A10-A13 og A14 og A15. solarVoltSensor har fået A9 27.824 Bytes 
// Ver 2.1.6 Nye stik til RF, Digitale porte til styring og dioder, samt et LCD display til at se info på.  30.950 Bytes 
// Har skervet dato og lidt sende stuff i displayet.
// Update 13-02-2015
// Rettet rød test diode til også at tænde når der køres up/down
// Motor commando kommer nu dobbelt så hurtige. Da den modtages 2 gange i setdet for 1.
// kig lidt på om panelvalues kan læses parallelt i stedet for 3 gange efter hinanden. Done.
// Update 16-02-2015
// Lavet print med extern 5 volt til relæer og lyssensorer. De bruger 200mA.
// Indsat buck converter inden selve Arduino mega2560, så den kun får 7 volt i stedet for 12. Så bliver den interne 7805 ikke så varm.
// Update 17-02-2015
// ReadLightSensorUpDownEastWest har fået en lyssensor LightAverage, så den ikke opdaterer så ofte når der ikke er sol. Og dermed sparer strøm.
// 29.898 Bytes
// Update 20-02-2015
// Lavet en optokopler til at måle spændingen på panelet, da laderegulatoren havde fælles pluds. Så kunne jeg ikke bruge den gamle spændingsdeler.
// Har lavet en case med forskellige volt områder der passer til optokopleren, da den ikke er linær. 33.898Bytes
// Update 22-02-2015
// Har slettet LCD lib igen, og alle lcd commands. OG tilføjet en hel masse (200) if sætninger for at få volt til at vise den rigtige værdi. Kun ikke finde en formel der virkede hele vejen op.
// 57.174 Bytes RAM 2579 
// Update 26-02-2015
// Har rettet trackupdate så den ikke runder 60 når der blir lagt f.eks 3 min til 59 min. Har samtidigt givet den ver 2.1.8
// Og ryddet lidt op i variabler, samt slettet nogle comments  56.232 Bytes
// Update 02-03-2015
// Har lavet sensor typen om fra byte til int, så den får en større værdi. Desuden har jeg lavet alle sensore om til LDR i stedet for phototransistor. 
// Værdien i beregningen er forøget fra 6 til 40 på East-Weat, men bíbeholdt på UP-Down da den er kritisk og giver lun max hvis der står inden for 2+- . 
// Tracker kører nu kun mellem 6 og 23 og GoHomeEast er også tilføjet kl 23.00 og kan køre i 2 min.  55.734 Bytes
// Update 06-03-2015
// Har som forsøg smidt Readbuttons op under RNF24L01_Panel sidst i funktionen, da den kørte 3 gange ReadButtons i gennem før nye data blev modtaget. 
// Og lavet en if sætning der sammenligner UD og EW og derefter sætter tiden hvis de begge er i rette position. Ser ud til at virke.
// Update 31-03-2015
// Manuel styring får første ret når der trykkes på knapperne fra displayet. Dvs. Autotrack disables i det tidsrum knappen trykkes ind.
// drawString=0 er blevet slettet i loop, da den ikke viste Manuel styring.
// Kanel 70 ændret til 125
// Update 26-04-2015
// Ændret tidspunkt hvor den kører mod ØST igen. 
// Update 15-05-2015
// Tilføjet parameter til service menuen, så der kan stilles på differenceEastWest og differenceUpDown. 58.134 Bytes
// Update 23-05-2015
// Måler også om natten nu, samt springer ReadLightSensorUpDownEastWest over hvis lightaverage < 600
// Rettet et par default værdier 56.170 Bytes 5549 (67%)  2643 local bytes free
// Update 16-06-2015
// Har fjernet Optokopler og erstattet den med alm. voltage divider 25 volt. Koden til denne er nu fjernet. Har monteret 2 dip switches der kan afbryde strømmes 
// til east & west motor hvis den kører for langt.
// Ny version 2.2.0              27.648 Bytes 3101 (37%)  5091 local bytes free
// Update 05-07-2015
// Har fået en RTC mere, der skal sættes på panelet, så den ikke glemmer tiden.
// Ny version 2.2.1
// Update 13-07-2015
// RTC virker på nu på modtager enheden. Har også tilføjet at Up/Down kun skal køre ml 10 og 18 (så strøm kun bruges når det bliver tilføjet om dagen)
// Men dipswitch UP er gået i stykker, så up/down er disabled igen.
// Update 20-07-2015
// Rep af dipswitch UP, koden tilføjet igen.
// Tilføjet NextTrackOffset justering da tiden ikke må blive større end NextTrackOffset.
// WestDown viser stadig for højt selvom alle 4 er blevet justeret ned i følsomhed.
//  if (LightAverage < 500 ) er blevet sat ned fra 600 til 500. Efter justering.
// Update 21-07-2015
// UP/Down taget fra igen...det larmer for meget i for lang tid. Venter til jeg har en trapezgevindstang. SetdifferenceUpDown   = 130
// Update 20/9-2015
// Radiodelen er tilpasset den ny mini remote. Og der er derfor lavet en ny case med anden crc værdi 1281.
// I Miniremoten sættes værdierne som normalt men her i modtageren læses disse værdier ikke ind. Dvs de bibeholder deres værdi.
// goHomeToEast er også rettet til kl 21:00, i stedet for 22:00
// Update 11/10-2015
// Tilføjet ny protokol i radiodelen 1310, så MiniRemoten også kan læses Amp, Volt, Temperatur uden at ændre parameter i panelet.
// Har også tilføjet 12mm Trapezgevind, men stigningen er stadig meget langsom. Automatis up/down er stadig diasbled, så kun manuel styring er mulig.
// Update 18/10-2015
// Version 2.2.2
// Har ændret lidt i goHomeToEast() da jeg ikke syntes den var hurtig nok til at aflæse status af kontakten. I loop kører den som while i stedet og selve kaldet er lavet som if else.
// Række følgen på lyssensorerne er også lavet om, da jeg har drejet printet. Måske det hjælper på den WD sensor der var lidt for "glad"
// Update 29/10-2015
// Har lavet goHomeToEast() tilbage til det oprindelige da jeg ikke syntes det nye hjalp og den fumlede i while løkken. Den døde lige som.
// Update 10/11-2015
// Har disablet denne funktion goHomeToEast();     den irriterrede mig. Da endstop ikke rigtig virker.
// Update 14/11-2015
// U=2         
// V=18volt   16.93  19.3  19.5   18.5  17.5   17.8  17.6   
// A=5.1      5.2     3.2   3.5   3.5    3.5   1.2    0.8
// Update 13/05-2016
// Har tilføjet følgende til ReadLightSensorUpDownEastWest() funktionen under "if (LightAverage < 500 )" 
//   StopMotorEW();  // Stop motor, så den ikke fortsætter hvis der kommer en sky
//   ResetRelayEW(); // Reset relæ så det ikke trækker strøm hele tiden.    
// Da motor ellers kunne fortsætte hvis der kom en sky forbi midt i en afsagt kommando om at gå mod West
// Og enabled goHomeToEast kl 21:00 igen.
// Update 17/05-2016
// har tilføjet en tid på visningen af debug info i ReadLightSensorUpDownEastWest, så det kun vises hvert andet sekund.
// Version 2.2.3
// Update 22/05-2016
// Har rettet i koden til " if (LightAverage < 500 ) " så tiden bliver sat inden den hopper ud af løkken.
// Update 10/7-2016
// Version 2.2.4
// Har rettet i koden til " if (LightAverage < 650 ) " da den kører rundt heletiden i regnvejr.
// Update 20/7-2016
// SetdifferenceEastWest=1; Dette skulle give en hystrese der kører til mindre end 1, men når 0 er opnået sættes SetdifferenceEastWest igen til 30 
// Tilføjet SoftReset() når Panelet går hjem kl 21:00
// Update 2/8-2016
// Version 2.2.4-1 (-1 betyder at koden kun er rettet lidt og sagtens kan køre med modsvarende versions nummer i Displayet, altså inden for 2.2.4 )
// Panelet står og kører tilbage når det har fundet difference=1 og nogle gange står det og kører frem og tilbage 2-3 gange ? det kan være min SetdifferenceEastWest=1 der gør dette.
// rettet if (EWinPosition && UDinPosition) til if (EWinPosition) da up/down ikke er tilsluttet endnu og derfor aldrig havner i denne tilstand.
// Panelet står stadig og kører lidt frem og tilbage efter rettelsen. Prøver at sætte SetdifferenceEastWest=5 i stedet.  Det ser ud til at have hjulpet. Rammer spot on 0
// Update 13-08-2016
// Ny version 2.2.5 Denne version har UP/Down motor tilsluttet. SetdifferenceUpDown   = 30 default i Display. Og her.



// Time lib
#include "Time.h"
// RTC lib
#include "DS1302RTC.h"
// Show memory lib
#include "MemoryFree.h"
// Interrupt lib
#include <avr/sleep.h>
#include <avr/power.h>
#include <SPI.h>
//Radio module NRF24L01+
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"


//Define solartracker ports, variables and static
//Digital ports
// 0 , 1 reserved for serial com
const byte dipSwitchEast     = 41;  // Endstop East
const byte dipSwitchWest     = 42;  // Endstop West
const byte MotorUDOn         = 43;  // relæ motor2 on/off til op/ned
const byte MotorMoveUpDown   = 44;  // relæ motor2 styring op/ned af panelet
const byte MotorMoveEastWest = 45;  // relæ motor1 øst/vest styring af rammen
const byte MotorEWOn         = 46;  // relæ motor1 on/off til øst/vest 
const byte dipSwitchDown     = 47;  // Endstop Down
const byte dipSwitchUp       = 40;  // Endstop Up


//Analog ports (DB9 stik 10,11,12,13,14,15)
int solarVoltSensor     = A9;   //Lille blå print
// Gamle værdier
//int lightSensorWestDown = A10;  //Orange kabel, Lille blå print 
//int lightSensorEastDown = A11;  //Blå kabel                     
//int lightSensorEastUp   = A12;  //Grøn kabel                    
//int lightSensorWestUp   = A13;  //Blå+hvid kabel                
int lightSensorEastDown = A10;  //Blå kabel                     
int lightSensorWestDown = A11;  //Orange kabel, Lille blå print 
int lightSensorWestUp   = A12;  //Blå+hvid kabel                
int lightSensorEastUp   = A13;  //Grøn kabel                    
int solarTempSensor     = A14;  //Blå+hvid kabel                
int solarCurrentSensor  = A15;  //Brun og hvid kabel            
// +5V Hvid+Orange   
// GND Brun



//Buttons and Potmeters
int LightAverage = 0;  //Lavet global for at kunne læse den i displayet
// rem 21/7-2015 static char ButtonsData[4] = "000";

//Global Light Variables. Send via RF24
int lightSensorEastUpValue    = 0;
int lightSensorWestUpValue    = 0;
int lightSensorEastDownValue  = 0;
int lightSensorWestDownValue  = 0;


//Global Sensor Variables
float Voltage     = 0.0;
float Amps        = 0.0;
float Temperature = 0.0; 
// int mappedVoltValue = 0;


//Global Track Variables
int NextTrackOffset = 0; // Indeholder realtime minutter, der skal trækkes fra waitMinutesToNextTrackUpdate så tiden blir 0. Og der kan opdateres.
int NextTrackOffsetANDwaitMinutesToNextTrackUpdate = 0; //NextTrackOffset + waitMinutesToNextTrackUpdate 


// ServiceMenu parametre
int SensorThreshold   = 0;              //Sunsensor, is messuring how much sun there should be before adjusting panel
int TempValue         = 0;              //Temperatur value
int VoltAmpSampleRate = 60;             //Samplerate for volt and amp
int waitMinutesToNextTrackUpdate = 1;   //This value is in minutes 1 equal one minut 5 eual 5 minuttes. In my application it is Set to 3 or 5 minutes
int SetdifferenceEastWest = 30;         //Tolerancen på hvornår panelet skal dreje (hvis sensorværdien er mere end 40 i forskel fra East til West) 
int SetdifferenceUpDown   = 30;         //Tolerancen på hvornår panelet skal vippe (hvis sensorværdien er mere end 10 i forskel fra up til down) 
int prevsec=0;                          //indeholder forrige værdi af sekunder

//Global Date Variables
byte BadRTC     = 0;        // Check if RTC i working or not working =0
bool Sommertid  = false;   


//Global Bolean 
bool ButtonMotoEWOnState      = 1; // 1= stop
bool ButtonMotoUDOnState      = 1; // 1= stop
bool ButtonMotorMoveEastState = 0;
bool ButtonMotorMoveWestState = 0;
bool East = HIGH; //Active high
bool West = LOW;  //Active Higi
bool Down = HIGH; //Active High
bool Up   = LOW;  //Active High
bool Run  = HIGH; //Active High
bool Stop = LOW;  //Active LOW
bool dipSwitchWestState = 0;
bool dipSwitchEastState = 0;
bool dipSwitchUpState   = 0;
bool dipSwitchDownState = 0;


//String variables
byte drawString         = 0;  //used for different type of strings to be displayed, when different actions accure


// Set up nRF24L01 radio on SPI bus plus pins 9 & 53 kan godt ændres til pin 10 eller andet.
//50 1-Miso ORANGE  2 +5V
//52 3 SCK  GRØN    4 Mosi  51  LILLA
//   5 RST          6 GND
//PRINT LAYOUT:
//1= GND  2=VCC
//3=CE    4=CSN
//5=CSK   6=MOSI
//7=MISO  8=IRQ
RF24 radio(49,48);  //50,51,52,4,
const uint64_t pipes[2] = { 0xDEDEDEDEE7LL, 0xDEDEDEDEE9LL };
boolean RadioLink = false;  // is there a radiolink
char *val;
char SendPayload[32] = "";
char RecvPayload[32] = "";
int crc=0;


// Define pin RTC1302 
// DS1302:         CE pin    -> Arduino Digital 2  RST  MEGA 39
// DS1302:         I/O pin   -> Arduino Digital 3  DAT  MEGA 38
// DS1302:         SCLK pin  -> Arduino Digital 4  CLK  MEGA 40
// DS1302:         Remember 5V and not 3,3V
// Init the DS1302 
// Set pins:  RST, DAT,CLK
DS1302RTC RTC(32, 34, 36); 


//********************
//Initialize and setup
//********************
void setup(void) 
{
  //Initialize ports and variables  
  analogReference(DEFAULT); //INTERNAL1V1 = 1.1V INTERNAL2V56 = 2.56V , DEFAULT = 5 volt

  pinMode(38,                OUTPUT);  // LED så jeg kan se om det er gået godt
  pinMode(13,                OUTPUT);  // LED så jeg kan se om det er gået godt
  pinMode(53,                OUTPUT);  // SPI CS bruges ikke, men skal være sat til OUTPUT 
  pinMode(MotorEWOn,         OUTPUT);  // Motor1 on-off
  pinMode(MotorMoveEastWest, OUTPUT);  // East 0, West 1
  pinMode(MotorUDOn,         OUTPUT);  // Motor2 on-off
  pinMode(MotorMoveUpDown,   OUTPUT);  // Up 0,  Down 1
  pinMode(dipSwitchWest,     INPUT);    
  pinMode(dipSwitchEast,     INPUT);    
  pinMode(dipSwitchUp,       INPUT);    
  pinMode(dipSwitchDown,     INPUT);  


  digitalWrite(38, HIGH); // blå test diode


  StopMotorUD();
  StopMotorEW();

  tmElements_t tm;   //Get the whole string from RTC      

  //Init goHomeToEast first time; 

  digitalWrite(MotorMoveEastWest, East);  //Remmes ud ved test af programmet, da kontakten ikke findes og det blir en uendelig løkke.
  digitalWrite(MotorEWOn, Run);           //Motor1 kør
//  do
//   {
//    dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch East 1 = pressed = end stop
//   }
//  while(dipSwitchEastState < 1);  // Hop videre når kontakt går høj = end stop

  digitalWrite(MotorMoveEastWest, West);  //Relæ sættes i off position, så den ikke trækker strøm i standby
  drawString = 0; // nulstiller drawString så den er klar til nye actions der skal op på skærmen.  
  StopMotorEW();
  StopMotorUD();

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

  
  //Set watch
  // rem 21/7 -2015 time_t myTime;
  // rem 21/7 -2015 myTime = RTC.get();



  // init startdate
  time_t t = now();
  minute(t);  // returns the minute for the given time t 
  NextTrackOffset = minute(t);
  Serial.println("Ver SolTracker_til_Panel_ver 2.2.5");
  
  
  // kunstig dato. Kun til test
  //  setTime(1, 55, 50, 21, 7, 2015); 


  // init value for calc- men er ikke sikker på det er nødvendigt
  Voltage=0.0;
  Amps=0.0;
  Temperature=0.0; 
//  mappedVoltValue=0;


  //NRF24L01+ begynder init her    
  printf_begin();

  radio.begin();
  radio.setDataRate(RF24_250KBPS); //speed	RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.setPALevel(RF24_PA_HIGH);  //Set Power Amplifier (PA) level to one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setChannel(125);           //channel	Which RF channel to communicate on, 0-127
  
  radio.enableDynamicPayloads();
  radio.setRetries(5,15);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableAckPayload();

  // Skal se sådan ud når Panel er RX
  radio.openWritingPipe(pipes[0]);    // Open different pipes when writing. Write on pipe 0, address 0
  radio.openReadingPipe(0,pipes[1]);  // Read on pipe 0, as address 1
  //RX_ADDR_P0-1	 = 0xdedededee7 0xc2c2c2c2c2
  //TX_ADDR	  = 0xdedededee9


  radio.startListening();
  radio.printDetails();

  Serial.println("SETUP DONE !");

  // Så meget fri RAM er der tilfbage
  Serial.print("Free RAM: ");  Serial.println( freeMemory() );

  ButtonMotoEWOnState          = 1; //1=Stop
  ButtonMotoUDOnState          = 1; //1=Stop 
  ButtonMotorMoveEastState     = 1;
  waitMinutesToNextTrackUpdate = 1;
  VoltAmpSampleRate            = 60;
  SensorThreshold              = 0;

  dipSwitchWestState = 0;
  dipSwitchEastState = 0;
  dipSwitchUpState   = 0;
  dipSwitchDownState = 0;
  delay(5000); // Time to watch if the setup was OK
}

// ************************************
// Funktion til omskrivning af integer til char
// ************************************
void GetString(char IntString[6] ,int i)
{
    Serial.print("i konvertering = ");
    Serial.println(i);    
  // Brug denne konvertering fra int til char i stedet 
    IntString[0] = (i % 10000)/1000 +'0';
    IntString[1] = (i % 1000)/100 +'0';
    IntString[2]=  '.';
    IntString[3]=  (i % 100)/10 +'0';
    IntString[4]=  (i % 10)/1 +'0';
}

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
void NRF24L01_ReceiveFromDisplay()
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
  int okval; 
  int Error=0;

//  Serial.println("-------Start Receive Mode ---------------- ");

  if ( radio.available() ) // er der radiolink
   {
    RadioLink = true;
// rem 21/7-2015    bool done = false;
     while (radio.available())
      {
	len = radio.getDynamicPayloadSize();
	radio.read( RecvPayload, len );
      }
    Serial.println("Radio link");
    RecvPayload[len] = 0;
   } //if
 else 
   {  
     Serial.println("INGEN radio link"); // Er ca 3 gange så hurtig som display delen
     return;
   }


 for (int i = 0; i < 32; i++) 
   DataPayload[i]=RecvPayload[i];

  CRC_Check(len); //Kald crc check med længden på RecvPayload

  Serial.println();
  Serial.print("RecvPayload = ");   Serial.println(RecvPayload);
  Serial.print("DataPayload = ");   Serial.println(DataPayload);
  Serial.print("Den nye CRC er nu   = ");   Serial.println(crc);
  Serial.println(); 
 
  Serial.println("------------slut paa 1.st Receive Data Mode delen -------  ");   
  Serial.println();


// rem 21/7-2015  time_t t = now();
  
  DataStringtoSD = "";    
 
  if ((crc==1966)||(crc==1280)) //1966 eller 1280
   {  
     // Sender de 4 LightValues til Display
     int lightparameter[5]={lightSensorEastUpValue, lightSensorWestUpValue, lightSensorEastDownValue, lightSensorWestDownValue, NextTrackOffsetANDwaitMinutesToNextTrackUpdate}; // 4 værdier af sammt type pakkes til een string. og så en NextTrackUpdate
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
     int lightparameter[5]={lightSensorEastUpValue, lightSensorWestUpValue, lightSensorEastDownValue, lightSensorWestDownValue, NextTrackOffsetANDwaitMinutesToNextTrackUpdate}; // 4 værdier af sammt type pakkes til een string. og så en NextTrackUpdate
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
     float parameter[3]={Voltage,Amps,Temperature}; // 2 værdier af sammt type pakkes til een string
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

  while ( !ACKdone ) 
   {
      // First, stop listening so we can talk
      radio.stopListening();
      radio.openWritingPipe(pipes[1]);    //Så kan der sendes TX 15-01-2015
      radio.openReadingPipe(0,pipes[0]);  //Så kan der sendes TX 15-01-2015    


      // Send the last CRC back 
      radio.write(SendPayload,len);
      Serial.print("Sent response. "); Serial.println(SendPayload);   

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
         StopMotorEW(); //ellers stopper motoren ikke hvis data går i hegnet
         StopMotorUD(); //ellers stopper motoren ikke hvis data går i hegnet  
         Error=Error+1;
         if (Error >4) return;
      }
    else
     {
      // Hent response, sammenlign det i en case
      len = radio.getDynamicPayloadSize();
      radio.read( RecvPayload, len );
      RecvPayload[len] = 0;

      // hvad kom der ind
      printf("Got response size=%i value=%s\n\r",len,RecvPayload);
      Serial.print("R:");
      Serial.print(RecvPayload);
      Serial.println();
 
      val = strtok (RecvPayload,","); // Hiver variable ud af stringen (som char)
      okval= atoi(val);               // laver char om til en integer

      Serial.print("okval= ");
      Serial.println(okval);

        
      // check at det er det korrekte check ciffer der hører til den rigtige string
   switch (crc)
       {
    case 1280: if (okval==333) // sender kun Lysværdi tilbage ...som ER gjort
         {
           ACKdone=true; 
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: ButtonMotoUDOnState = atoi(val);  
               break;
               case 2: ButtonMotorMoveEastState = atoi(val);               
               break;
               case 3: waitMinutesToNextTrackUpdate = atoi(val);               
               break;
               case 4: VoltAmpSampleRate = atoi(val);               
               break;
               case 5: SensorThreshold = atoi(val);               
               break;
               case 6: SetdifferenceEastWest = atoi(val);               
               break;
               case 7: SetdifferenceUpDown = atoi(val);               
               break;
              }
             } //while
            count=0;
            digitalWrite(13, HIGH);
            digitalWrite(38, LOW);
            Serial.println("Data modtaget fra Diaplay ACK 333 (SKRIVER DATA SetMotor)");

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

         }//if (okval==333)
    break;         

    case 1281: if (okval==444) // sender kun Lysværdi tilbage ...som ER gjort
         {
           int Tempval   = 0;
           ACKdone=true; 
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: ButtonMotoUDOnState = atoi(val);  
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
            digitalWrite(13, HIGH);
            digitalWrite(38, LOW);
            Serial.println("Data modtaget fra Diaplay ACK 444 (SKRIVER DATA SetMotor)");

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

         }//if (okval==444)
    break;         

    case 1309: if (okval==222)   // Sætter motor parameterne og sample værdier
          { 
           ACKdone=true;   
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: ButtonMotoUDOnState = atoi(val);  
               break;
               case 2: ButtonMotorMoveEastState = atoi(val);               
               break;
               case 3: waitMinutesToNextTrackUpdate = atoi(val);               
               break;
               case 4: VoltAmpSampleRate = atoi(val);               
               break;
               case 5: SensorThreshold = atoi(val);               
               break;
               case 6: SetdifferenceEastWest = atoi(val);               
               break;
               case 7: SetdifferenceUpDown = atoi(val);               
               break;

              }
             } //while
            count=0;
            digitalWrite(13, HIGH);
            digitalWrite(38, LOW);
            Serial.println("Data modtaget fra Diaplay ACK 222 (SKRIVER DATA SetMotor)");

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

          } // if (okval==222)
    break;


    case 1310: if (okval==555)   // Sætter motor parameterne og sample værdier
          { 
           int Tempval   = 0;
           ACKdone=true;   
           val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
           ButtonMotoEWOnState = atoi(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
           while ((val = strtok (NULL, ",")) != NULL)
            { 
             ++count;
             switch (count)
              { 
               case 1: ButtonMotoUDOnState = atoi(val);  
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
            digitalWrite(13, HIGH);
            digitalWrite(38, LOW);
            Serial.println("Data modtaget fra Diaplay ACK 555 (SKRIVER DATA SetMotor)");

            // check bit ButtonMotorMoveEastState og lav ButtonMotorMoveWesttState til det modsatte
            if  (ButtonMotorMoveEastState == 0)
              ButtonMotorMoveWestState=1;
            else
              ButtonMotorMoveWestState=0;

          } // if (okval==555)
    break;



    case 1966: 
         if (okval==111)   // hvis ok er data valide og jeg behøver ikke gensende datoen
          { 
           ACKdone=true; 
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
            digitalWrite(13, HIGH);
            digitalWrite(38, LOW);
            Serial.println("Data modtaget fra Diaplay ACK 111 (SKRIVER DATA SetTime)");
          
            time_t t = now();
            Serial.println("Slave tiden inden justering: ");       
            Serial.print("Tiden(t)= ");       
            Serial.print(year(t));   Serial.print(":");       
            Serial.print(month(t));  Serial.print(":");       
            Serial.print(hour(t));   Serial.print(":");       
            Serial.print(minute(t)); Serial.print(":");        
            Serial.println(second(t));       

            setTime(timer,minutter,sekunder,dag,maaned,aar); // sætter tiden internt
            RTC.set(now());                                  // overfører tiden til RTC enheden
            Serial.println("Tiden efter justering: ");       
            Serial.print("Tiden(t)= ");       
            Serial.print(year(t));   Serial.print(":");       
            Serial.print(month(t));  Serial.print(":");       
            Serial.print(hour(t));   Serial.print(":");       
            Serial.print(minute(t)); Serial.print(":");        
            Serial.println(second(t));       
            Serial.print("Hentet Master tid er= ");                           
            Serial.print(aar);   Serial.print(":");       
            Serial.print(maaned);  Serial.print(":");       
            Serial.print(timer);   Serial.print(":");       
            Serial.print(minutter); Serial.print(":");        
            Serial.println(sekunder);       
            NextTrackOffset = minute(t);  // så er Nexttrack også opdateret.
        } //  if (okval==111)
    break;
         } // switch (crc)

    if (!ACKdone) // Ved ikke om denne skal slettet igen ??
        { 
         StopMotorEW(); //ellers stopper motoren ikke hvis data går i hegnet
         StopMotorUD(); //ellers stopper motoren ikke hvis data går i hegnet  
        }

        DataPayload[len] = 0;  // Put a zero at the end for easy printing
        RecvPayload[0]   = 0;  // Clear the buffers          
        DataPayload[0]   = 0;  // Clear the buffers  
      }//else
    }//while

  RecvPayload[0] = 0;  // Clear the buffers
  DataPayload[0] = 0;  // Clear the buffers  
  Serial.println("-------------------Finish med at hente og sende.  Klar til ny loop.---------------------");         
  Serial.println();       
  // læser digitale værdier fra kontakterne til manuel motorstyring
  ReadButtons();                 
  
}


// ************************************
// Wakes Arduino from sleep mode on pin D21 
// ************************************
void pin2Interrupt(void)
{
  /* This will bring us back from sleep. */
  
  /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  detachInterrupt(0);
}

// ************************************
// Puts Arduino into sleep mode. 40mA less.
// Can be wake with int2, pin21
// ************************************
void enterSleep(void)
{
  
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(2, pin2Interrupt, LOW);
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();
  
  /* The program will continue from here. */
  
  /* First thing to do is disable sleep. */
  sleep_disable(); 
}

// ************************************
// Can do a software reset of the Arduino print
// ************************************

void SoftReset()
{
asm volatile ("  jmp 0");  // Restarts program from beginning but does not reset the peripherals and registers
}



// ************************************
// Printer 3 tomme linier i serial monitor
// ************************************
void printline() 
{
Serial.println();                                 
Serial.println();
Serial.println();                                 
}

// ************************************
// Serial monitor. Printer alle værdierne ud
// Volt, amp, amphr, totamp, avgamp, powernow, energi, og energitime
// ************************************
void PrintValues() 
{
   printline(); // Serial.println(); x 3
   Serial.print("VOLTAGE : ");
   Serial.print(Voltage);
   Serial.println(" Volt");

   Serial.print("CURRENT : ");
   Serial.print(Amps);
   Serial.println(" Amps");
   
   Serial.print("CURRENT Hours: ");
//   Serial.print(amphr);
   Serial.println(" Amphr");

   Serial.print("Total Current : ");
//   Serial.print(totamps);
   Serial.println(" Amps");

   Serial.print("Average Amps : ");
//   Serial.print(avgamps);
   Serial.println(" Amps");
   
   Serial.print("POWER Now :");
   Serial.print(Voltage*Amps);
   Serial.println(" Watt");
   
   Serial.print("ENERGY CONSUMED :");
//   Serial.print(ProducedEnergyHours);
   Serial.println(" Watt-Hour");
   
   Serial.print("LoopCount ");
//   Serial.println(LoopCount);


   Serial.print("Temperature ");
   Serial.println(Temperature);



//   Virker ikke i dette scope.
//   Serial.print("EnergyTime ");
//   Serial.println(EnergyTime);
   
   Serial.println(" "); // print the next sets of parameter after a blank line
//   delay(1000);

   printline(); // Serial.println(); x 3
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
  Serial.println();
  Serial.println("--------Start ReadButtons------------- ");  //hvis ikke denne er her, vil uret ikke vises   
//  Read the contacts

//  ButtonMotorMoveEastState = digitalRead(ButtonMotorMoveEast); // kontakt op
//  ButtonMotorMoveWestState = digitalRead(ButtonMotorMoveWest); // kontakt ned
 
//  motor1 run E/W, motor2 run U/D, kontakt U/D-E/W, NextTracktime, VoltAmpSampleRate, Sensorthreshold, SetdifferenceEastWest, SetdifferenceUpDown
//  parameterbool[3]={ButtonMotoEWOnState,ButtonMotoEWOnState,ButtonMotorMoveEastState}; 

  
    if (ButtonMotoEWOnState ==1) StopMotorEW(); //ellers stopper motoren ikke
    if (ButtonMotoUDOnState ==1) StopMotorUD(); //ellers stopper motoren ikke   
    
    // Først East West kontakten
    if ( ButtonMotoEWOnState == 0)  // hvis fjederkontakten er trykket ned så gør følgende
     {
//       Serial.println("--------MotorEWOn Kontakten er aktiv ------------- ");
     
        switch (ButtonMotorMoveEastState) //Was the switch set to East ?
         {
         case 0:
           //do something when var equals 1
            Serial.println("Manuel, Going East");
            goToEastManuel();
           break;
         }

        switch (ButtonMotorMoveWestState) //Was the switch set to West ?
         {
         case 0:
           //do something when var equals 1
            Serial.println("Manuel, Going West");
            goToWestManuel();
           break;
         }
     }

    // og så til Up Down kontakten 
    if ( ButtonMotoUDOnState == 0)  // hvis fjederkontakten er trykket ned så gør følgende
     {
        switch (ButtonMotorMoveEastState) //Was the switch set to East/UP ?
         {
         case 0:
            Serial.println("Manuel, Going Up");
            goUpManuel();
           break;
         }

        switch (ButtonMotorMoveWestState) //Was the switch set to West/DOWN ?
         {
         case 0:
            Serial.println("Manuel, Going Down");
            goDownManuel();
           break;
         }
     }
}

// ************************************
// Get the lightsensor value on 4 Analog pins
// and save them in Local variables
// ************************************
void ReadLightSensorUpDownEastWest()
{

  Serial.println();
  Serial.println("--------Start ReadLightValues------------- ");  //hvis ikke denne er her, vil uret ikke vises   
  lightSensorEastUpValue   = 0;
  lightSensorWestUpValue   = 0;
  lightSensorEastDownValue = 0;
  lightSensorWestDownValue = 0;

  // Hvis der er manuel styring, så hop ud af denne funktion.   
  if (( ButtonMotoEWOnState == 0)||( ButtonMotoUDOnState == 0))  // hvis fjederkontakten er trykket ned så gør følgende
   {
    lightSensorEastUpValue   = 999;    // Så kan jeg se i displayet at den ikke læser lightsensor's
    lightSensorWestUpValue   = 999;
    lightSensorEastDownValue = 999;
    lightSensorWestDownValue = 999;
    return;
   } 


  bool UDinPosition=false, EWinPosition=false;
 
  int sampletime=10;
   for(int i=0;i<sampletime;i++)  // taking 50-100 samples from sensors with a inerval of 2msec and then average the samples data collected
    {
      lightSensorEastUpValue   += analogRead(lightSensorEastUp);    //read the voltage from the sensor
      lightSensorWestUpValue   += analogRead(lightSensorWestUp);
      lightSensorEastDownValue += analogRead(lightSensorEastDown);
      lightSensorWestDownValue += analogRead(lightSensorWestDown);
      delay(2);
    }
   
   lightSensorEastUpValue=lightSensorEastUpValue/sampletime;   // hiv gennemsnittet så blir det lidt mere præcist
   lightSensorWestUpValue=lightSensorWestUpValue/sampletime;
   lightSensorEastDownValue=lightSensorEastDownValue/sampletime;
   lightSensorWestDownValue=lightSensorWestDownValue/sampletime;
   
  // bliver fastlage i servicemenuen. Default = 0tte
  // SensorThreshold = analogRead(TolerancePot)/4; // read potentiometer and put it in Tolerance. It will adjusts the threshold light difference before the solartracker moves

   // Average values
   int lightSensorUpAverage   = (lightSensorEastUpValue + lightSensorWestUpValue) / 2;     //Average Top
   int lightSensorDownAverage = (lightSensorEastDownValue + lightSensorWestDownValue) / 2; //Average Down
   int lightSensorEastAverage = (lightSensorEastUpValue + lightSensorEastDownValue) / 2;       //Average East
   int lightSensorWestAverage = (lightSensorWestUpValue + lightSensorWestDownValue) / 2;       //Average West

   // Avarage fra alle sensorer, giver tal der kan måle om lyset er for svagt. 200= en gråvejrsdag i november uden sol og bare skyer.
   LightAverage = (lightSensorUpAverage + lightSensorDownAverage + lightSensorEastAverage + lightSensorWestAverage) / 4; 

   int DifVertikal = lightSensorUpAverage - lightSensorDownAverage;   // the difference between the 2 sunvalues Up and Down
   int DifHoizontal = lightSensorEastAverage - lightSensorWestAverage;// the difference between the 2 sunvalues East and West

   
   int differenceEastWest = abs(DifHoizontal);                             // 'abs' is the absolute value, the result is always positive.
   int differenceUpDown   = abs(DifVertikal);                              // 'abs' is the absolute value, the result is always positive.



// NextTrackOffset                                = selve real tiden
// waitMinutesToNextTrackUpdate                   = tiden jeg kan indstille fra 1-15 minutter
// NextTrackOffsetANDwaitMinutesToNextTrackUpdate = tiden der er til næste track update ( realtiden lagt sammen med tiden jeg selv kan indstille )

   // Kontrol af tiden, om den ryger forbi 60
   NextTrackOffsetANDwaitMinutesToNextTrackUpdate = NextTrackOffset+waitMinutesToNextTrackUpdate;
   if (NextTrackOffsetANDwaitMinutesToNextTrackUpdate > 59)  // sker ved f.eks NextTrackOffset=57 og waitMinutesToNextTrackUpdate er 3 minutter
     NextTrackOffsetANDwaitMinutesToNextTrackUpdate=NextTrackOffsetANDwaitMinutesToNextTrackUpdate-60; // ved at trække 60 fra, blir resultatet 0 eller mere end 0



/* der skal kigges på dette her, for den kan ikke køre ordentligt ved waitMinutesToNextTrackUpdate = 2 minutter.
   Serial.print("CASE  NextTrackOffsetANDwaitMinutesToNextTrackUpdate "); Serial.println(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
  
   switch (NextTrackOffsetANDwaitMinutesToNextTrackUpdate)
    { 
      case 58: if (waitMinutesToNextTrackUpdate == 3) { Serial.println("CASE 58 = 3 ");  
                   NextTrackOffsetANDwaitMinutesToNextTrackUpdate=1;} 
      break;
      case 59: if (waitMinutesToNextTrackUpdate == 2) { Serial.println("CASE 59 = 2 "); 
                   NextTrackOffsetAND
                 =59;} 
      break;
    }
    
*/


  
   time_t t = now();  // skal deklareres ellers kan den ikke se tiden 
   
if (((second(t) % 2 == 0) && (prevsec != second(t))))  //hvert andet sekund ..hvis den er spot on. så vises det ikke så ofte, og programmet blir hurtigere   
  {
   /***************************DEBUG INFO  ***************/

   // TIMESTAMP
     Serial.println();       
     Serial.print("Tiden(t)= ");       
     Serial.print(hour(t));   Serial.print(":");       
     Serial.print(minute(t)); Serial.print(":");        
     Serial.println(second(t));       
 
    //show SensorThreshold
      Serial.println("if (-1*SensorThreshold > DifVertikal || DifVertikal > SensorThreshold) ");
      Serial.print("SensorThreshold is ");     Serial.println(SensorThreshold);
      Serial.println();            
      Serial.print("DifVertikal = ");    Serial.println(DifVertikal);            
      Serial.println();            
      Serial.print("-1*SensorThreshold = ");   Serial.println((-1*SensorThreshold));      
      Serial.print("LightAverage is ");  Serial.println(LightAverage);      
 
      Serial.print("NextTrackupdate + NextTrackOffset: "); Serial.println(waitMinutesToNextTrackUpdate+NextTrackOffset);   
      Serial.print("NextTrackOffsetANDwaitMinutesToNextTrackUpdate: ");  Serial.println(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
      Serial.print("Minute: ");  Serial.println(minute(t));    
      Serial.print("EU: ");  Serial.println(lightSensorEastUpValue);
      Serial.print("WU: ");  Serial.println(lightSensorWestUpValue);
      Serial.print("ED: ");  Serial.println(lightSensorEastDownValue);
      Serial.print("WD: ");  Serial.println(lightSensorWestDownValue);
      Serial.print("SetdifferenceEastWest: ");  Serial.println(SetdifferenceEastWest);
      Serial.print("SetdifferenceUpDown: ");  Serial.println(SetdifferenceUpDown);

      Serial.print("UD Average: ");  Serial.println(differenceUpDown);
      Serial.print("EW Average: ");  Serial.println(differenceEastWest);
      
      Serial.print("lightSensorUpAverage    = "); Serial.println(lightSensorUpAverage);
      Serial.print("lightSensorDownAverage  = "); Serial.println(lightSensorDownAverage);
      Serial.print("lightSensorEastAverage  = "); Serial.println(lightSensorEastAverage);            
      Serial.print("lightSensorWestAverage  = "); Serial.println(lightSensorWestAverage);
      Serial.print("lightSensorAverage i alt  = "); Serial.println(LightAverage);
      
      Serial.print("differenceUpDown  : ");    Serial.println(differenceUpDown);
      Serial.print("differenceEastWest: ");  Serial.println(differenceEastWest);      
      Serial.println();            
      Serial.print("dipSwitchUpState  : ");  Serial.println(digitalRead(dipSwitchUp));   
      Serial.print("dipSwitchDownState: ");  Serial.println(digitalRead(dipSwitchDown));   
      Serial.print("dipSwitchEaseState: ");  Serial.println(digitalRead(dipSwitchEast));   
      Serial.print("dipSwitchWestState: ");  Serial.println(digitalRead(dipSwitchWest));  
      prevsec=(second(t));        
  } //if 

   if (LightAverage < 650 ) 
    {
     Serial.println("No sun light,  Powersave mode. Exit Calculations ");
     drawString =17; // "NO SUN.  Powersave mode "....(Powersave mode måske )
     StopMotorEW();  // Stop motor, så den ikke fortsætter hvis der kommer en sky
     ResetRelayEW(); // Reset relæ så det ikke trækker strøm hele tiden.    

     //  husk at sætte timeren inden exit, ellers går det galt med tiden
     time_t t = now();    
     NextTrackOffset = minute(t);     

     /***************************DEBUG INFO  ***************/
     Serial.println("***************************DEBUG INFO BEFORE EXIT ***************"); 
     Serial.println(" "); 
     Serial.print("differencen East/West was ");
     Serial.println(differenceEastWest);        
     Serial.print("differencen Up/Down was ");
     Serial.println(differenceUpDown);        

     Serial.println("----------------------");
     Serial.print("Realtime = ");                                       Serial.print(hour(t)); Serial.print(":");Serial.print(minute(t)); Serial.print(":"); Serial.println(second(t)); 
     Serial.print("NextTrackOffset = ");                                Serial.println(NextTrackOffset);
     Serial.print("waitMinutesToNextTrackUpdate = ");                   Serial.println(waitMinutesToNextTrackUpdate);
     Serial.print("NextTrackOffsetAND = "); Serial.println(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
     Serial.println(" "); 
     Serial.print("When minute(t) ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print("NextTrackOffset + waitMinutesToNextTrackUpdate "); Serial.print(NextTrackOffset+waitMinutesToNextTrackUpdate);  Serial.println(" then UPDATE  *** ");
     Serial.print("When minute(t) ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print("NextTrackOffsetANDwaitMinutesToNextTrackUpdate "); Serial.print(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);  Serial.println(" the same  *** ");
     Serial.println(" "); 
     Serial.println("And now EXIT  ");     
     Serial.println(" ");     
     Serial.println(" ");   
     return;          //hop tilbage til hvor den blev kaldt
    }  


//********* Her kommer den rigtige beregning på panelets hældning *********************

//  if (-1*SensorThreshold > DifVertikal || DifVertikal > SensorThreshold) // Start- check om -tol>dif ELLER dif>tol, else change Up Down (vertical angle)
//  { //1
//    Serial.println("SUN Sensivity is now within range. The UD calc can continue");
    if ((lightSensorUpAverage < lightSensorDownAverage) && (differenceUpDown > SetdifferenceUpDown))  // aller bedst inden for 2, men kan være svært, da den vil korrigerer hele tiden
   { //2   Husk når solen skinner mest på den øverste sensor skal panelet gå ned(tilbage mod fladt) og når solen skinner mest på den nederste sensor, skal panelet gå op (løftes mod 90 grader)
     time_t t = now();
     Serial.println("lightSensorUpAverage < lightSensorDownAverage ...jeg Kan justerer UP (hvis tiden passer) ");
     if ((NextTrackOffsetANDwaitMinutesToNextTrackUpdate) == (minute(t)))  //Hvis NextTrackOffset og waitminuttesnexttrackupdate er det samme som tiden, så kør UP
 //      if ((hour(t)>=10) && (hour(t)<18))       //Kør kun hvis klokken er mellem 10 og 18
       { //3 
        drawString =13;
        Serial.println("Automatic, Going to Up");
        goUp(); //  rem until trapezgevind is mounted 
       } //3 
   } //2
  else if ((lightSensorUpAverage > lightSensorDownAverage) && (differenceUpDown > SetdifferenceUpDown)) //aller bedst inden for 2, men kan være svært. Og den opdaterer hele tiden. STRØM !.
  {//4
    time_t t = now();
    Serial.println("lightSensorUpAverage > lightSensorDownAverage ...jeg Kan justerer DOWN (hvis tiden passer)");
    if ((NextTrackOffsetANDwaitMinutesToNextTrackUpdate) == (minute(t)))   //Hvis NextTrackOffset og waitminuttesnexttrackupdate er det samme som tiden, så kør DOWN
 //    if ((hour(t)>=10) && (hour(t)<18))       //Kør kun hvis klokken er mellem 10 og 18
      {//5
       drawString =14;
       Serial.println("Automatic, Going Down");
       goDown(); // rem until trapezgevind is mounted 
      } //5    
  } //4
  else if (differenceUpDown <= SetdifferenceUpDown)  // hvis forskellen på de 2 værdier er mellem 0 og 60 så er de så godt som lige, og nexttrack kan udregnes  
      {//6
        UDinPosition=true; //bit sat til Nexttime ok
        drawString =15;
        Serial.println("YES, U/D is in position ");
        Serial.print("differencen up/down er ");
        Serial.println(differenceUpDown);        
        StopMotorUD();                          
        ResetRelayUD();                      // Reset relæ så det ikke trækker strøm hele tiden.        
      }//6
     
  // } //1 ************************* Slut- check om differencen er inden for tolerancen, else change Up Down (vertical angle)




  if (-1*SensorThreshold > DifHoizontal || DifHoizontal > SensorThreshold) // Start- check om differencen er inden for tolerancen, else change East West (Horizontal angle)
  {
    Serial.println("SUN Sensivity is now within range. The EW calc can continue");
    if ((lightSensorEastAverage < lightSensorWestAverage) && (differenceEastWest > SetdifferenceEastWest)) //omkring 40 så kører den ikke så ofte
    {
     Serial.println("(lightSensorEastAverage < lightSensorWestAverage...jeg justerer West.. hvis tiden er korrekt");
     time_t t = now();
     if ((NextTrackOffsetANDwaitMinutesToNextTrackUpdate) == (minute(t)))  //Hvis NextTrackOffset og waitminuttesnexttrackupdate er det samme som tiden, så go west
      {
       drawString =4;
       Serial.println("Automatic, Going to West");
       goToWest();
       SetdifferenceEastWest=5;                              // Dette skulle give en hystrese der kører til mindre end 5, men når 5 er opnået sættes SetdifferenceEastWest igen til 30 
      }  
    }
  else if ((lightSensorEastAverage > lightSensorWestAverage) && (differenceEastWest > SetdifferenceEastWest))
  {
    Serial.println("(lightSensorEastAverage > lightSensorWestAverage ... jeg justerer East....hvis tiden er korrekt)");
    time_t t = now();
   if ((NextTrackOffsetANDwaitMinutesToNextTrackUpdate) == (minute(t)))   //Hvis NextTrackOffset og waitminuttesnexttrackupdate er det samme som tiden, så go east
    {
     drawString =5;
     Serial.println("Automatic, Going to East");
     goToEast();
    }     
  }
  else if (differenceEastWest <= SetdifferenceEastWest)  // hvis forskellen på de 2 værdier er mellem 0 og 60 så er de så godt som lige, og nexttrack kan udregnes
   {
     EWinPosition=true; //bit sat til Nexttime ok
     drawString =6;
     Serial.println("YES, E/W is in position ");
     Serial.print("differencen East/West er ");  Serial.println(differenceEastWest);        
     StopMotorEW();                          // ingen update når sensor er lige og NextTrackOffset bliver = time now minutter
     ResetRelayEW();                         // Reset relæ så det ikke trækker strøm hele tiden.        
     SetdifferenceEastWest=30;               // Dette skulle give en hystrese der kører til 0, men når 0 er opnået sættes SetdifferenceEastWest igen til 30 
   }// else if
  }// Start- check om differencen er inden for tolerancen, else change East West (Horizontal angle)

  
  
  // Nexttrack=time(t) ...hvis begge er i rette position
  //if (EWinPosition && UDinPosition)   // brug denne når UP/Down virker
  if (EWinPosition)                     // brug denne når kun E/W virker
  { 
    time_t t = now();    
    NextTrackOffset = minute(t);     
    Serial.println(" ");  Serial.println("NextTrackOffset er nu RESAT og ny periode af NextTrack kan begynde");      
    ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.        
    ResetRelayUD();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }

/***************************DEBUG INFO  ***************/
    Serial.println(" "); 
    Serial.println(" "); 
     Serial.println("--------The calc is over--------------");
    Serial.print("differencen East/West is ");
    Serial.println(differenceEastWest);        
    Serial.print("differencen Up/Down is ");
    Serial.println(differenceUpDown);        

    Serial.println("----------------------");
    Serial.print("Realtime = ");                                       Serial.print(hour(t)); Serial.print(":"); Serial.print(minute(t)); Serial.print(":"); Serial.println(second(t)); 
    Serial.print("NextTrackOffset = ");                                Serial.println(NextTrackOffset);
    Serial.print("waitMinutesToNextTrackUpdate = ");                   Serial.println(waitMinutesToNextTrackUpdate);
    Serial.print("NextTrackOffsetANDwaitMinutesToNextTrackUpdate = "); Serial.println(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
    Serial.println(" "); 
    Serial.print("When ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print(NextTrackOffset+waitMinutesToNextTrackUpdate);  Serial.println(" then UPDATE  *** ");
    Serial.print("When ");  Serial.print(minute(t)); Serial.print(" = "); Serial.print(NextTrackOffsetANDwaitMinutesToNextTrackUpdate);  Serial.println(" the same  *** ");
    Serial.println(" "); 
    Serial.print("drawString: ");    Serial.println(drawString);
    Serial.println(" ");     

}

// ************************************
// Get the solar Voltage, Current and Temp on Analog pins
// and save them in Globals variables
// Scale: 0 to 20 volt DC
// Her læser jeg den analoge indgang 15 gange og tar gennemsnittet.
// ************************************
void ReadPanelValues()
{
   Serial.println("--------Start ReadPanelValues------------- ");  //hvis ikke denne er her, vil uret ikke vises   
//  int  sampletime=60; // sætter alle sample værdier så alle får samme antal samples
   
    float sampleA    = 0.0;
    float sampleB    = 0.0;
    float sampleC    = 0.0;
    Voltage         = 0.0;
    Amps            = 0.0;
    Temperature     = 0.0; 

  
//Read Volt value *****************************
    for(int i=0;i<VoltAmpSampleRate;i++)  // taking 50-100 samples from sensors with a inerval of 2msec and then average the samples data collected
     {
      sampleA+=analogRead(solarVoltSensor);     //read the voltage from the sensor
      sampleB+=analogRead(solarCurrentSensor);  //read the voltage from the current sensor  5A=187mv/A 30A=66mv/A
      sampleC+=analogRead(solarTempSensor);     //read the voltage from the temp sensor          
      delay(2);
     }

    sampleA=sampleA/VoltAmpSampleRate;   // hiv gennemsnittet
    Serial.print(sampleA); Serial.println(" sampleA = ");

    Voltage = map( sampleA, 0, 1023, 0, 2500 ); // måler mellem 0 og 24 volt    
    Voltage=Voltage/100; // da map er i tusinder
    
    Serial.print(Voltage); Serial.println(" Volt from panel ");
    


// Read Current value *****************************

    sampleB=sampleB/VoltAmpSampleRate;   // hiv gennemsnittet    
    if (sampleB < 513) sampleB = 513;     
    if (sampleB > 715) sampleB = 513;	

    Amps = map( sampleB, 513, 715, 0, 8500 ); 
    Amps = Amps / 1000; // for at få det i milliamperer
//    Serial.print(Amps); Serial.println(" Amperer");

// Read Temperatur value *****************************

    sampleC=sampleC/VoltAmpSampleRate;   // hiv gennemsnittet

    Temperature = sampleC * 5.23;   // 5.23 bliver det da 5 volt giver for lav temperatur.
    Temperature /= 1024.0; 

    // print out the voltage
     Serial.print(Temperature); Serial.println(" Temperature in volts");
 
    // Convert to temperature in degrees C
    Temperature = (Temperature - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)
     Serial.print(Temperature); Serial.println(" degrees C");
    if ((Temperature <= -40) || ( Temperature >= 120))  //Så er det uden for måleområdet
      Temperature=0.0;
}

// ************************************
// små rutiner der kan blive brugbare senere
// ************************************
void blandet()
{ 
   printline(); // Serial.println(); x 3                                
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

void ResetRelayUD()
{ 
  digitalWrite(MotorMoveUpDown, Up);      //Relæ sættes i off position, så den ikke trækker strøm i standby
  digitalWrite(MotorUDOn, Run);           //Relæ Motor2 kør
  digitalWrite(MotorUDOn, Stop);          //Low = stop
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
  drawString = 3;
  digitalWrite(MotorMoveEastWest, East);        // Vend Relæ East/West mod Øst
  digitalWrite(MotorEWOn, Run);                 // Start MotorRelæ EW, Panel kører
  dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchEastState == 1)
  {
   StopMotorEW();
   ResetRelayEW();                                // Reset relæ så det ikke trækker strøm hele tiden.            
   ResetRelayUD();                                // Reset relæ så det ikke trækker strøm hele tiden.         
  }

}

//********End goHomeToWest********************

//**********************************
// MotorEastWest Goto West
//**********************************
void goToWest()
{
  drawString = 5;  // "Automatic. Going to West"
  digitalWrite(MotorMoveEastWest, West); //  
  digitalWrite(MotorEWOn, Run); 
  dipSwitchWestState = digitalRead(dipSwitchWest); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchWestState == 1)
  {
    StopMotorEW();
    drawString =8;                       // " West END stop reached  "
    ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }
}
//*******End goToWset*********************

//**********************************
// MotorEastWest Goto East
//**********************************
void goToEast()
{
  drawString = 4;  //"Automatic. Going to East"
  digitalWrite(MotorMoveEastWest, East);  
  digitalWrite(MotorEWOn, Run); 
  dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchEastState == 1)
  {
    StopMotorEW();
    drawString =7;                       // " East END stop reached  "
    ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }
}
//******End goToEast**********************

//**********************************
// MotorEastWest Goto West Manuel
//**********************************
void goToWestManuel()
{
  drawString = 2;
  digitalWrite(MotorMoveEastWest, West); //  
  digitalWrite(MotorEWOn, Run); 
  dipSwitchWestState = digitalRead(dipSwitchWest); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchWestState == 1)
  {
    StopMotorEW();
    drawString =8;                       // " West END stop reached  "
    ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }
}
//*******End goToWsetManuel*********************

//**********************************
// MotorEastWest Goto East Manuel
//**********************************
void goToEastManuel()
{
  drawString = 1;
  digitalWrite(MotorMoveEastWest, East);  //
  digitalWrite(MotorEWOn, Run); 
  dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchEastState == 1)
  {
    StopMotorEW();
    drawString =7;                       // " East END stop reached  "
    ResetRelayEW();                      // Reset relæ så det ikke trækker strøm hele tiden.        
  }
}
//******End goToEastManuel**********************

//**********************************
// Stop motorEastWest
//**********************************
void StopMotorEW()
{
  digitalWrite(MotorEWOn, Stop); //Low = stop
}
//*********End stopMotor*******************

//*******************************************Motor Up Down********************************
//**********************************
// Shutdown when its windy
//**********************************
void goHomeDown()
{
  drawString = 16;                                 // "   Shutdown. To windy   "
  digitalWrite(MotorMoveUpDown, Down);  
  digitalWrite(MotorUDOn, Run); 

  dipSwitchDownState = digitalRead(dipSwitchDown); // read dipswitch Down. 1 = pressed = end stop
  if (dipSwitchDownState == 1)
  {
    StopMotorUD();
    ResetRelayUD();                                // Reset relæ så det ikke trækker strøm hele tiden.        
  }
}
//********End goHomeDown********************

//**********************************
// MotorUpDown Goto Up
//**********************************
void goUp()
{
  drawString = 13;
  digitalWrite(MotorMoveUpDown, Up); //  
  digitalWrite(MotorUDOn, Run); 
  dipSwitchUpState = digitalRead(dipSwitchUp); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchUpState == 1)
   {
    StopMotorUD();
    drawString =9;    
    ResetRelayUD();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }
}
//*******End goUp*********************

//**********************************
// MotorUpDown Goto Down
//**********************************
void goDown()
{
  drawString = 14;
  digitalWrite(MotorMoveUpDown, Down);  
  digitalWrite(MotorUDOn, Run); 
  dipSwitchDownState = digitalRead(dipSwitchDown); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchDownState == 1)
  {
    StopMotorUD();
    drawString =10;
    ResetRelayUD();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }
}
//******End goDown**********************

//**********************************
// MotorUp Goto Up Manuel
//**********************************
void goUpManuel()
{
  drawString = 11;
  digitalWrite(MotorMoveUpDown, Up); //  
  digitalWrite(MotorUDOn, Run); 
  dipSwitchUpState = digitalRead(dipSwitchUp); // read dipswitch west 1 = pressed = end stop
  if (dipSwitchUpState == 1)
  {
    StopMotorUD();
    drawString =9;
    ResetRelayUD();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  }
}
//*******End goUpManuel*********************

//**********************************
// MotorUpDown Goto Down Manuel
//**********************************
void goDownManuel()
{
  drawString = 12;
  digitalWrite(MotorMoveUpDown, Down);  
  digitalWrite(MotorUDOn, Run); 
  dipSwitchDownState = digitalRead(dipSwitchDown); // read dipswitch Up 1 = pressed = end stop
  if (dipSwitchDownState == 1)
  {
    StopMotorUD();
    drawString =10;
    ResetRelayUD();                      // Reset relæ så det ikke trækker strøm hele tiden.            
  } 
}
//******End goDownManuel**********************

//**********************************
// Stop motorUpDown
//**********************************
void StopMotorUD()
{
  digitalWrite(MotorUDOn, Stop); 
}
//*********End stopMotorUpDown*******************

//**********************************
// Sommer og Vintertid
//**********************************
void SommerVinterTid()
{
  time_t t = now();
  // hvis tiden er ml 22:00 og 22:02 så gå hjem til øst. Sommertid.

   
 if (Sommertid)                           // Sommertid ?
  if (month(t)>=10)
  if (day(t)==29)                         // Skift til vintertid hvis klokken passer
   if (hour(t)==23)
    if (minute(t)==59)
     if (second(t)==59) 
     {
      setTime(23, 0, 0, day(t), month(t), year(t)); 
      RTC.set(now());                     // set the RTC from the system time

      Sommertid=false;                    // Skift til vintertid.
     }

  if (!Sommertid)                          // Sommertid ?
  if (month(t)==3)                         // Skift til sommertid hvis klokken passer
   if (day(t)==25)
   if (hour(t)==23)
    if (minute(t)==59)
     if (second(t)==59) 
     {
      setTime(0, 59, 59, day(t), month(t), year(t)); 
      RTC.set(now());                     // set the RTC from the system time

      Sommertid=true;                     // Skift til vintertid.
     }  
  
}
//*********End stopMotorUpDown*******************

//**********************************
// The loop where everything happens
//**********************************
void loop(void) 
{
  SommerVinterTid();  //Skifter mellem vinter og sommertid
  
  //tracker kører kun ml 6:00 og 21:59.   HUSK RTC i display kører på sommertid, dvs den er 1 time forud om vinteren.
  time_t t = now();
  ReadLightSensorUpDownEastWest();  // Læser både up/down og East/West. Motorerne blir styret af solen her. 
  ReadPanelValues();                // læser analoge værdier volt,amp, temp osv

  // hvis dato og tid er ml 24:00 og 00:01 så gå hjem til øst. Solen er bag taget 19:45 21/7-2016
  if (hour(t)==21 && minute(t) == 0)                  // Kan det gøres på 1 min ?. // rettet til 0, 28/10-15
   {
     dipSwitchEastState = digitalRead(dipSwitchEast); // read dipswitch west 1 = pressed = end stop
     while(dipSwitchEastState == 0)                   // Alting stopper når denne while løkke kører. Kører indtil kontakten bliver High
       goHomeToEast();                                // Er klokken mere end 21:00 så kør hjem mød øst. Og reset relæerne.
 
     StopMotorEW();                                   // Jeg vil være helt sikker på at motoren stopper.  
     ResetRelayEW();                                  // Reset relæ så det ikke trækker strøm hele tiden.            
     ResetRelayUD();                                  // Reset relæ så det ikke trækker strøm hele tiden.         
     SoftReset();                                     // Vi starter på en frisk. Alle variabler resettes // 10/5 solen er bag taget kl 19:30
   }

  // Denne del skal køre i panelet
  NRF24L01_ReceiveFromDisplay();  // modtager data fra Displayet *færdig      
  
  // læser digitale værdier fra kontakterne til manuel motorstyring
  //ReadButtons();  Lagt i slut af NRF24L01_ReceiveFromDisplay();               
  

  digitalWrite(13, LOW);  // slukker dioden
  digitalWrite(38, HIGH); // slukker den blå diode

}

