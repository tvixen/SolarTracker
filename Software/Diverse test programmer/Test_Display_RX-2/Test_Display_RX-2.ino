//*************************************************************************
// Arduino IDE Version: 1.8.5
//
// Author: Tim Milgart
//
// Program for: Solar Display /control Panel. 
// Function:    Test TX/RX function
//
// Start Date: 07.12.2018
// License: Tim Milgart
//*************************************************************************

// sakset fra SolTracker_til_Display_ver 2.3.0

// Sender SendToPanel(2) " ,1,0,1,20,99,99,1309, " som er motor, og får ",12.33,45.66,78.99,101,1309," tilbage
// Som er Volt, Amp, Temp og DrawString.
// Sender SendToPanel(1) " ,16,30,59,01,01,2015,1966, " som er dato og tid, og får "999,999,999,999,1966," tilbage
// som er LightSensor Values.
// Sender SendToPanel(3) " ,,11,12,13,14,15,16,1280, " som er uden dato og tid, og får "999,999,999,999,1966," tilbage
// som er LightSensor Values. Så sendes dato&tid kun en gang i døgnet.
// Har tilføjet 3 dioder der kan vise status på forbindelsen.
// GRØN BLINK datatransmission kører som den skal
// RØD Der har været pakketab.
// BLÅ lyser hvis batteri pakken kommer under X volt.
// Slut på ver 2.1.0 og ny starter på ver 2.1.1 her vil display funktioner blive tilføjet.
// Ver 2.1.1 inkluderet RTC funktionen 21.228 Bytes  Virker fint.
// Ver 2.1.2 inkluderet Display funktion 28.066 Bytes.  Med draw og Servicemenu 51.585 Bytes. Virker fint.
// Ver 2.1.2 Har givet pin 49 og 45 hhv til pin 7 og 6 CE og CS på NRF24L01+ modulet for at holde ledningerne samlet. Virker fint.
// Ver 2.1.3 inkluderet ReadButtons(). 53.038 Bytes. Virker fint (har afprøvet service menu)
// Har tilføjet reset af TotalError via East vippekontakten i servicemenuen.
// Ver 2.1.4 Har fjernet serial.print i NRF24L01_SendToPanel
// Har tilføjet LightSensorValues, så de kan aflæses live fra NRF24L01_SendToPanel
// Har tilføjet Volt Amp Temp og drawstring, så de kan aflæses live fra NRF24L01_SendToPanel 51.784 Bytes
// Den er nu længere tid om at sende ...fordi serial.print er fjernet i NRF24L01_SendToPanel, så jeg måtte
// indsætte et delay(60) for den ikke skulle kløjes i det. Det virker
// Volt, Amp, Temperatur virker også nu.
// Radio data bliver alle sat til 0, hvis signalet forsvinder. 52.478 Bytes Free RAM 4316
// Har tilføjet WaitWifiText, så jeg kan se om RTC kører som den skal i starten.
// Har tilføjet rettelse af timer i servicemenuen og fjernet internet settings. 53.588 Bytes  Free RAM 3964 
// Sat level op på RF modulet fra LOW til HIGH med 3db (dobbelt op) for at se om det hjælper på fejl transmissioner.
// Remmet delay i loop mellem radio.powerup og menu. Den kører alligevel fuld hastighed.
// Fjernet dipswitch variablerne.
// Update: 08-02-2015
// Ver 2.1.5 53.608 Free RAM 3959
// Sat level op på RF modulet fra HIGH til MIN med 6db (2x dobbelt ned) for at se om det hjælper på fejl transmissioner.
// MIN rækker 3 meter
// LOW rækker 7 meter
// Tilføjet den sidste streng fra knapperne til RF, så motor1 og motor2 kan styres fra display. 54.808 Bytes Free RAM 3968
// Update 11-02-2015
// Rykket dioderne da port stikket er monteret igen. De hedder nu 10,11,12  blå rød grøn
// Update 13-02-2015
// Sender nu dato ved opstart og kl 6.30, ellers sender motor, nexttrack, samplerate hver gang, jeg får så volt, amp temperatur på lige og lightsensorvalues på ulige. 
// Update 15-02-2015
// Rettet i Trackupdate 55.156 Bytes og også i panelversionen 2.1.6 så det passer sammen.
// Fjernet serial.print i showdate, trackupdate showvolt, showcurrent osv osv.
// givet ny versions nummer 2.1.6 54.948Bytes
// Trackupdate skriver 11:61 når tiden går over 1 time.
// Update 02-03-2015
// Har rettet loop så den sender tiden hvert 10 minut i 5 sekunder ...så ser vi om det retter problemet med trackupdate.
// Har desuden loddet RNF24L01 i stedet for det lille stik.
// Update 06-03-2015
// Ny version 2.1.7
// Har ændret lidt i draw for at gøre den hurtigere i menuen. Ved at skrive u8g2.setPrintPos(205,119); i stedet for  u8g2.print("ED");  u8g2.drawStr( 205, 119, "ED");
// OG har flytte alle kald ned i selve draw. Det har givet 0.3 sekunder, som er super godt, men gir en meget rodet kode og dårligt overblik.
// Har indtil videre beholdt procedurene i selve koden, med de kan reelt slettes.
// Update 31-03-2015
// Kanal ændret fra 70 til 125.
// Rød led blinker nu ved fejl. Men i service menu kan man stadig se total antal fejl.
// Update 15-05-2015
// Ny version 2.1.8
// Tilføjet parameter til service menuen, så der kan stilles på SetdifferenceEastWest og SetdifferenceUpDown. Og justeret 3 default værdier 55.074 Bytes
// Samt ændret sun meter til LightAverage i stedet for Amps.
// Update 16-06-2015
// Ny version 2.2.0 slettet lidt i koden, mest rem afsnit.Resset Sunline fra 300 til 500
// Update 13-07-2015
// Ny version 2.2.1 Tiden sendes kun i starten, da modtager enheden også har fået RTC.
// Update 21-07-2015
// SetdifferenceUpDown   = 130 indtil trapezgevind er monteret. Er monteret nu men stigningen er ikke høj nok.
// Update 12-09-2015
// Batteri er blevet indbygget 7,2V Lipo .... volt ref er justeret, Battery = 7.2V (2x3,6V)
// Update 14-11-2015
// Ny version 2.2.2
// Under service menuen tænker jeg at lave en live graf, så man kan se hvornår solen har været højst. Med opdatering hvert sekund el 10 sekund. 
// Update 17-05-2016
// ServiceMenu opdateres med AvargeLight samt har rykket det hele lidt til højre.
// Ny version 2.2.3
// Update 20-07-2016
// Ny version 2.2.4 da Panelet er blevet opdateret til 2.2.4
// Update 13-08-2016
// Ny version 2.2.5 Denne version har UP/Down motor tilsluttet. SetdifferenceUpDown   = 30 default
// Update 23-04-2017
// Ny compiler fra 1.6.12 har opdateret time.h så min DS1302RTC ikke virker. Derfor har jeg taget den gamle time.h og omdøbt den til time_old.h og lagt den i mappen sammen med DS1302RTC filerne.
// Inde i selve DS1302RTC.h har jeg lavet #include <time_old.h> i stedet for #include <time.h> ellers virker det ikke mere. Samt under C:\Users\Cool\Documents\Arduino\libraries\Time
// også lagt filen time_old.h  Husk dette fremadrettet!
// 1. Under mappen C:\Users\Cool\Documents\Arduino\libraries\Time lægges time_old.h
// 2. Under mappen C:\Users\Cool\Documents\Arduino\libraries\DS1302RTC  lægges time_old.h
// 3. Rediger DS1302RTC.h til #include <time_old.h> i stedet for #include <time.h>
// Update 24-04-2017
// Nyt Library er kommet in fra siden. Det er fra support forummet blevet oplyst at TimeLib.h kan de samme ting som tidligere time.h og den er kampatibel med 1.8.2 ... Så den bruger jeg i stedet for.
// Ligger i mappen "Time-master"
// Update 25-05-2017
// Ny version 2.2.8 Har tilføjet en funktion i servicemenuen der kan få panelet til at gå hjem, så kontakterne kan testes.
// Uploaded til board fra Cloud
// Update 16-05-2018
// Ny version 2.3.0 kan snakke med det nye panel som kører version mini_display. Der kan ved opstart vælges hvilket panel man vil connecte til.
// hvis kanppen holdes inde, forbindes til det store panel, ellers forbindes til mini panel.
// Update 27-05-2018
// Tilføjet 2 ekstra drawString 18 & 19. Der kan vise hår Hardware Endstop bliver aktiveret.
// Update 12-12-2018
// Tilføjet den nye version af U8g2lib.h, bare for at se om den var hurtigere og bedre
// Update 13-12-2018 if 
// Tilføjet ((!ButtonProgState)&& (!ButtonMotoEWOnState)) VoltAmpSampleRate=99; ... Hvis ButtonProg er aktiv sammen med fjederkontakt ButtonMotoEWOnState,
// bliver "HardwareEndStop=true" aktiv 13/12-2018 (når prog knappen trykkes ned samtidigt med man trykker på fjeder til East/West)
     

//#include "time_old.h"   //Denne ligger som en ekstra fil i directory DS1302RTC, ellers virker det ikke med de nye compilere
#include "TimeLib.h"      //Denne er det nye time library
#include "DS1302RTC.h"    // RTC lib
//#include "U8glib.h"     //removed old version 12/12-2018
#include "U8g2lib.h"      // Dispaly lib. Den nye version tilføjet 12/12-2018
#include "MemoryFree.h"   // Show memory lib
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"



//Define solartracker ports, valiables and static
//Digital ports
// 0 , 1 reserved for serial com
const byte MotorUDOn         = 3;  // relæ motor2 on/off til op/ned
const byte MotorMoveUpDown   = 3;  // relæ motor2 styring op/ned af panelet
const byte MotorMoveEastWest = 3;  // relæ motor1 øst/vest styring af rammen
const byte MotorEWOn         = 3;  // relæ motor1 on/off til øst/vest 
const byte DisplayOff        = 35; // går til A18 Display OFF  High=Display On, GND=Display off   
const byte DisplayFont       = 36; // går til A19 Display font 6*8 =  HIGH 8*8 = LOW
const byte DisplayReverse    = 37; // går til A20 Display reverse P/N  
const byte DisplayRD         = 24; // går til RD. Skal altid være High
#define RED                    11  // Pin 11 = Red diode
#define GREEN                  12  // Pin 11 = Green diode
#define BLUE                   10  // Pin 11 = Blue diode



//Buttons and Potmeters
const byte ButtonMotorEWOn     = 46;  // fjeder kontakt nr 2 fra højre on=46. (45 gruges som +5v og sættes HIGH)
const byte ButtonMotorMoveEast = 43;  // 3 vejskontakt  ned=west(41)  midten(42)  op=EAST(43)
const byte ButtonMotorMoveWest = 41;  // 3 vejskontakt  ned=west(41)  midten(42)  op=EAST(43)
const byte ButtonMotorUDOn     = 48;  // fjeder kontakt nr 3 fra højre on=48. (47 gruges som +5v og sættes HIGH) Kan skifte f.eks display ml. 1 og 2. Eller anden funktion
const byte ButtonProg          = 44;  // Trykkontakt for at vælge eller programmere
int Tolerance    = 0;  // analogRead(TolerancePot)/4; // read potentiometer global foreløbig til test 15/11-2014
int LightAverage = 0;  //Lavet global for at kunne læse den i displayet




//Global incomming Analog ports (DB9 stik 10,11,12,13,14,15)
//int solarCurrentSensor     =   0;  // modatget fra Panel på A10 Orange kabel, Lille blå print 
int lightSensorEastUpValue   =   0;  // modatget fra Panel på A11 Blå kabel                     
int lightSensorWestUpValue   =   0;  // modatget fra Panel på A12 Grøn kabel                    
int lightSensorEastDownValue =   0;  // modatget fra Panel på A13 Blå+hvid kabel                
int lightSensorWestDownValue =   0;  // modatget fra Panel på A14 Blå+hvid kabel                
//int solarTempSensor        =   0;  // modatget fra Panel på A15 Brun og hvid kabel            
// +5V Hvid+Orange   
// GND Brun

int BatterySensor       = A0;  //Lille blå print der måler indkomne spænding til Mega2560 og viser det i servicemenuen.
int TolerancePot        = A1;  //potmeter til justering af east/west, up/down tolerance


//Global Sensor Variables
float Battery     = 0.0;
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
byte BadRTC     = 0;                  // Check if RTC i working or not working =0
int startyear   = 0;
int startmonth  = 0;
int startday    = 0;
int starthour   = 0;
int startminute = 0;
int startsecond = 0;
unsigned long currentmillis=0;         // gemmer antal millisekunder. range from 0 to 4,294,967,295
int viser       = 0;                   // used in a meter to show sun/volt
int ProgSeconds = 0;                   // sekunder der bruges til at programmere med, nedtælling
long LoopCount  = 0;                   // bruges til energi beregning, hvor mange gange har der været loop.
int NextTrackUpdate = 0;               // Minutes to nexttrackupdate realtime. This value is comming from Panel.
boolean SetNextTrackinSeconds = true;

// ServiceMenu parametre
int Menuvalg               = 0;
byte servicemenu           = 0;        //0= alm menu 1= service menu
int SensorThreshold        = 0;
int TempValue              = 0;
int VoltAmpSampleRate      = 60;
int SleepTime              = 1;
int waitMinutesToNextTrackUpdate = 1;  //This user value is in minutes. 1 equal one minut 5 eual 5 minuttes. Default is 2. 
int WifiOn                 = 0;        //Wifi On=1 Off=0. Bruger integer da den ellers ikke vil skrives ud i service menuen.
int SetdifferenceEastWest = 30;        //Tolerancen på hvornår panelet skal dreje (hvis sensorværdien er mere end 40 i forskel fra East til West)
int SetdifferenceUpDown   = 30;        //Tolerancen på hvornår panelet skal vippe (hvis sensorværdien er mere end 10 i forskel fra up til down)
int GraphTime             = 10;        // This user value is in seconds. Sets the time of live graph draw from the panel in Servicemenu2
char AmpLine[232]         = "";        // Set of values to show a graph for Amps
char VoltLine[232]        = "";        // Set of values to show a graph for Volts
int  LCDcount             =  0;        // Global counter for graph in servicemenu2
String DataStringtoLCD    = "";        // Part of a graph in servicemenu2
int BigpanelOn            =  0;        // Vælg panel stor elle mini . Bruger integer da den ellers ikke vil skrives ud i service menuen.

//Global Bolean 
bool ButtonMotoEWOnState =0;
bool ButtonMotoUDOnState =0;
bool ButtonMotorMoveEastState =0;
bool ButtonMotorMoveWestState =0;
bool East = HIGH; //Active high
bool West = LOW;  //Active Higi
bool Down = HIGH; //Active High
bool Up   = LOW;  //Active High
bool Run  = HIGH; //Active High
bool Stop = LOW;  //Active LOW
bool ButtonProgState    = 0;
bool Prog               = 0;
bool ServiceMenu2       = false;
bool GoHome             = false;   

//String variables
byte drawString         = 0;            //used for different type of strings to be displayed, when different actions accure

// Define pin RTC1302 
// DS1302:         CE pin    -> Arduino Digital 2  RST  MEGA 39
// DS1302:         I/O pin   -> Arduino Digital 3  DAT  MEGA 38
// DS1302:         SCLK pin  -> Arduino Digital 4  CLK  MEGA 40
// DS1302:         Remember 5V and not 3,3V
// Init the DS1302 
// Set pins:  RST, DAT,CLK
DS1302RTC RTC(39, 38, 40);

//Display driver til Hitachi display
// OLD U8GLIB_T6963_240X128 u8g(25, 26, 27, 28, 29, 30, 31, 32, 33, 22, 23, 24, 34); // MEGA2560 - porte 8Bit: D0..D7: 25-32, A0, A1, A3, A4, A2 
//NY
U8G2_T6963_240X128_2_8080 u8g2(U8G2_R0, 25, 26, 27, 28, 29, 30, 31, 32, /*enable= WR*/ 23, /*cs=*/ 33, /*dc=A0*/ 22, /*reset=*/ 34); // Connect RD with +5V, FS0 and FS1 with GND


// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 kan godt ændres til andre pins.
// Set Power Amplifier (PA) level to one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
// The power levels correspond to the following output levels respectively: NRF24L01: -18dBm, -12dBm,-6dBM, and 0dBm
//50 1-Miso RØD      2 +5V
//52 3 SCK  ORANGE   4 Mosi  51  BRUN
//   5 RST           6 GND
RF24 radio(45,49);
//const uint64_t pipes[2] = { 0xDEDEDEDEE7LL, 0xDEDEDEDEE9LL }; //BIGPanel Bliver sat i Initpanel();
const uint64_t pipes[2] = { 0xDEDEDEDEE1LL, 0xDEDEDEDEE2LL }; // WritingPipe, ReadingPipe For SmallPanel
boolean stringComplete = false;  // whether the string is complete
boolean RadioLink = false;  // is there a radiolink
char *val;
int count = 0;
char SendPayload[32] = "";
char RecvPayload[32] = "";
int crc=0;



// Bruges i loop
long counter_for_numbers_of_loops=0;
int  prevsec=0;
int Error=0;
long TotalError=0;


//********************
//Initialize and setup
//********************
void setup(void) 
{
 //Initialize ports and variables  
  analogReference(DEFAULT); //INTERNAL1V1 = 1.1V INTERNAL2V56 = 2.56V , DEFAULT = 5 volt
 
  pinMode(53,             OUTPUT);            // SPI CS bruges ikke, men skal være sat til OUTPUT 
  pinMode(BLUE,           OUTPUT);            // LED så jeg kan se om det er gået godt
  pinMode(RED,            OUTPUT);            // LED så jeg kan se om det er gået godt
  pinMode(GREEN,          OUTPUT);            // LED så jeg kan se om det er gået godt  
  pinMode(DisplayRD,      OUTPUT);                  
  pinMode(DisplayOff,     OUTPUT);
  pinMode(DisplayFont,    OUTPUT);
  pinMode(DisplayReverse, OUTPUT);  
  
  pinMode(ButtonMotorUDOn,     INPUT_PULLUP);  // fjeder kontakt nr 3 fra højre on=47. skal give +5v før det virker. 
  pinMode(ButtonMotorEWOn,     INPUT_PULLUP);  // fjeder kontakt nr 2 fra højre on=46. 
  pinMode(ButtonMotorMoveEast, INPUT_PULLUP);  // 3 vejskontakt  ned=west(41)  midten(stel)  op=EAST(43)  
  pinMode(ButtonMotorMoveWest, INPUT_PULLUP);  // 3 vejskontakt  ned=west(41)  midten(stel)  op=EAST(43)  
  pinMode(ButtonProg,          INPUT_PULLUP);  // Trykkontakt til programmering
  pinMode(MotorEWOn,         OUTPUT);          // Motor1 on-off
  pinMode(MotorMoveEastWest, OUTPUT);          // East 0, West 1
  pinMode(MotorUDOn,         OUTPUT);          // Motor2 on-off
  pinMode(MotorMoveUpDown,   OUTPUT);          // Up 0,  Down 1

  digitalWrite(DisplayRD,     HIGH);           // RD ReadData, skal altid være HIGH
  digitalWrite(DisplayOff,    HIGH);           // Tænder display
  digitalWrite(DisplayFont,    LOW);           // A19 Display font 6*8 = HIGH 8*8 = LOW
  digitalWrite(DisplayReverse, LOW);           // A20 Display reverse P/N 

  digitalWrite(BLUE,  HIGH);
  digitalWrite(RED,   HIGH);
  digitalWrite(GREEN, HIGH);  

  tmElements_t tm;   //Get the whole string from RTC      
  
  
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
    GoHome = false; //do not go home 
    
    Serial.println("Version: SolTracker_til_Display_ver 2.3.0 - Test_Display_RX");

    //Set watch
    time_t myTime;
    myTime = RTC.get();

    //Servicemenu til at programmere værdier
    Prog=0; 

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
u8g2.begin();
    //Lidt skærm utility
   // flip screen, if required
   // u8g2.setRot180();
   // set SPI backup if required
   // u8g2.setHardwareBackup(u8g_backup_avr_spi);
   // assign default color value
     u8g2.setColorIndex(255);     // white
     u8g2.setColorIndex(3);         // max intensity
    u8g2.setColorIndex(1);         // pixel on


  

   
   drawString=0;      // Ingen tekst


   // Spørg om Wifi skal være aktivt
   Serial.println( "Ask Panel commect ?" );     
   ButtonProgState=1; // Default uden Wifi
   drawString=0;      // Ingen tekst
   BigpanelOn=3;      //ingen panel valgt. 
   u8g2.firstPage();  
   do {
       PanelWaitText(); // service menu  
      } while( u8g2.nextPage() );
  

   Serial.print( "Panel is: "); Serial.println(BigpanelOn);
   u8g2.firstPage();  
   do {
       PanelWaitText(); // service menu  
      } while( u8g2.nextPage() );
//   delay(1500); // viser hvad der blev trykket i 1½ sek
    

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

 if (BigpanelOn==1)
  {                                         // Open pipe to the old Panel on the living room roof.
   radio.openWritingPipe(0xDEDEDEDEE7LL);   // Open different pipes when writing. Write on pipe 0, address 0
   radio.openReadingPipe(0,0xDEDEDEDEE9LL);  
   Serial.println("Big Panel init aktiv "); 
  }
 else
  {                                         // Open pipe to Nyt_Panel on the roof of the shed  
   radio.openWritingPipe(0xDEDEDEDEE1LL);   // Open different pipes when writing. Write on pipe 0, address 0
   radio.openReadingPipe(0,0xDEDEDEDEE2LL);  
   Serial.println("Mini Panel init aktiv "); 
  }

  radio.openWritingPipe(pipes[0]);   // Open different pipes when writing. Write on pipe 0, address 0
  radio.openReadingPipe(0,pipes[1]);  
  
  radio.startListening();
  radio.printDetails();

  Serial.println("SETUP DONE !");

  // Init datoen i panelet
  NRF24L01_SendToPanel(1);  // 1= set dato og tid (crc=1280) Lysværdi retur

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

  if (GoHome) // This will tell the panel to go home.
    waitMinutesToNextTrackUpdate= 99;

  switch (DataMode)
    {
      case 1:{ //   setTime(,16,30,59,01,01,2015,1966,); // sender tiden og får LightSensorValues tilbage
               //   Serial.println("Sending Date&Time Mode ");
              time_t t = now();
              sprintf(DateTime, ",%02d,%02d,%02d,%02d,%02d,%04d,%04d,",hour(t), minute(t), second(t),day(t),month(t),year(t),1966); //Set time
              DataStringtoSD=DataStringtoSD+DateTime;  //chars 26  
              NextCaseDataMode=DataMode;
             }
      break;
      case 2:{//  Sender motor1 run E/W, motor2 run U/D, kontakt U/D-E/W, NextTracktime, VoltAmpSampleRate, Sensortreshold, SetdifferenceEastWest, SetdifferenceUpDown
              // og får Volt, Amp, Temperatur, Drawstring tilbage 
              // DataStringtoSD = ",1,1,0,1,60,0,40,10,1309,"; //25 chars 
              bool parameterbool[3]={ButtonMotoEWOnState,ButtonMotoUDOnState,ButtonMotorMoveEastState}; // 3 værdier af sammt type (boolean) pakkes til een string
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
              DataStringtoSD=DataStringtoSD+1309;
              DataStringtoSD=DataStringtoSD+","; // husk at afslutte med ,
              NextCaseDataMode=DataMode;
             }
      break;
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

  Serial.print(" Outgoing "); Serial.println(DataStringtoSD);  // Bare så jeg lige kan kontrollere hvad der er i strengen default er den = (,1,1,1,2,60,0,1309,) 
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
//        Serial.println(SendPayload); // og så lige længden på strengen  
 
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
//         printf("Failed, response timed out.\n\r");
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

 Serial.print("incomming "); Serial.println(DataPayload);
       
        CRC_Check(len); //Kald crc check med længden på RecvPayload

        okval=crc;
        // Delay da den ellers bliver kvalt i hastighed efter fjernelsen af serial.print...desværre. default=(60)
        delay(60);

          
        // check at det er det korrekte check ciffer der hører til den rigtige string
        switch (NextCaseDataMode)
         {
           case 1: if (okval==1966)   // hvis ok er data valide og jeg behøver ikke gensende datoen
                    { 
                      ACKdone=true; 
//                     Serial.println("ACK OK from CRC, Display sending 111 to confirm");
                      radio.openWritingPipe(pipes[1]);    //Så kan der sendes TX 15-01-2015
                      radio.openReadingPipe(0,pipes[0]);  //Så kan der sendes TX 15-01-2015
                      radio.stopListening();
                      radio.write("111", 3);
                      radio.openWritingPipe(pipes[0]);    //Samme som i setup RX 15-01-2015
                      radio.openReadingPipe(0,pipes[1]);  //Samme som i setup RX 15-01-2015
                      radio.startListening();
                      digitalWrite(GREEN, LOW);   
                                           
                      val = strtok (DataPayload,","); //Hiver den første variable ud af stringen 
                      lightSensorEastUpValue= atof(val);  // konverterer over i Voltage som er en float (der kan laves beregninger på)
                      while ((val = strtok (NULL, ",")) != NULL)
                       {
                        ++countup;
                        switch (countup)
                         {
                          case 1: lightSensorWestUpValue = atoi(val);               
                          break;
                          case 2: lightSensorEastDownValue = atoi(val);               
                          break;
                          case 3: lightSensorWestDownValue = atoi(val);               
                          break;
                          case 4: NextTrackUpdate = atoi(val);
                         } //Switch
                       } //while

                    }
           break;
           case 2: if (okval==1309)   // hvis ok er data valide og jeg behøver ikke gensende motorværdi
                    { 
                      ACKdone=true; 
//                      Serial.println("ACK OK from CRC, Display sending 222 to confirm"); 
                      radio.openWritingPipe(pipes[1]);    //Så kan der sendes TX 15-01-2015
                      radio.openReadingPipe(0,pipes[0]);  //Så kan der sendes TX 15-01-2015                      
                      radio.stopListening();
                      radio.write("222", 3);
                      radio.openWritingPipe(pipes[0]);    //Samme som i setup RX 15-01-2015
                      radio.openReadingPipe(0,pipes[1]);  //Samme som i setup RX 15-01-2015
                      radio.startListening();
                      digitalWrite(GREEN, LOW);       

                      val = strtok (DataPayload,","); //Hiver den første variable ud af stringen
                      Voltage= atof(val);        // konverterer over i Voltage som er en float (der kan laves beregninger på)
                      while ((val = strtok (NULL, ",")) != NULL)
                       {
                        ++countup;
                        switch (countup)
                         {
                          case 1: Amps = atof(val);
                          break;
                          case 2: Temperature = atof(val);               
                          break;
                          case 3: drawString = atoi(val);               
                          break;
                         } //switch
                       } //while 
                    } 
           break;
           case 3: if (okval==1280)   // hvis ok er data valide. Uden dato, lysværdi retur
                   {             
                      ACKdone=true; 
//                      Serial.println("ACK OK from CRC, Display sending 333 to confirm");
                      radio.openWritingPipe(pipes[1]);    //Så kan der sendes TX 15-01-2015
                      radio.openReadingPipe(0,pipes[0]);  //Så kan der sendes TX 15-01-2015
                      radio.stopListening();
                      radio.write("333", 3);
                      radio.openWritingPipe(pipes[0]);    //Samme som i setup RX 15-01-2015
                      radio.openReadingPipe(0,pipes[1]);  //Samme som i setup RX 15-01-2015
                      radio.startListening();
                      digitalWrite(GREEN, LOW);  
                     
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
//          Serial.println("CRC IKKE FUNDET ..... RETURN");       
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
//            Serial.println("Time OUT");       
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
  u8g2.setFont(u8g_font_ncenB08);
  u8g2.drawStr(50, 126, "  -- Fejl i RTC modul -- "); // Nederst i midten
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
   Serial.println(" ------------ ReadButtons --------------------");  
   //Read the contacts
   ButtonMotoEWOnState =      digitalRead(ButtonMotorEWOn); //Fjederkontakt
   ButtonMotoUDOnState =      digitalRead(ButtonMotorUDOn); //Fjederkontakt   
   ButtonMotorMoveEastState = digitalRead(ButtonMotorMoveEast); // kontakt op
   ButtonMotorMoveWestState = digitalRead(ButtonMotorMoveWest); // kontakt ned
   


   if ((ButtonMotorMoveEastState == 1) && (ButtonMotorMoveWestState == 1)) // east/west kontakten skal være i midten. Så kan kommer service menuen
       servicemenu=1;
   else
     servicemenu=0;   
    
   

   ButtonProgState=digitalRead(ButtonProg);
   time_t t = now();


  if (!ButtonProgState)                            //Hvis  ButtonProg er aktiv, resettes TotalError. Husk at baggrundslyset til displayet kan gi forstyrrelser
     TotalError=0;

  if ((!ButtonProgState)&& (!ButtonMotoEWOnState))  //Hvis ButtonProg er false(aktiv = trykket ned) sammen med West eller East skal HardwareEndStop overwrite blive aktiv 13/12-2018
     VoltAmpSampleRate=99;
  else   
     VoltAmpSampleRate=0;

     
  if ((ButtonProgState == 0)&&(ButtonMotoEWOnState)) // ButtonMotoEWOnState må ikke være holdt nede sammen med ButtonProgState
  {
    time_t t = now();    
    ProgSeconds =  second(t);
    delay(250);
    if (Prog == 1) 
      Prog = 0;
    else 
      Prog = 1;
  }


  if ((Prog == 0) && (ProgSeconds + 10) >= second(t)) // er der gået mindre end 10 sek kan der programmeres
   {
    switch (Menuvalg) //Read the value of potmeter to chose submenu
    {
      // Husk der skal trækkes 1 fra i case. Så 6 rent faktisk er 7, da jeg starter ved 0 i stedet for 1 :)
     case 0:
        //Sunsensor
        SensorThreshold=TempValue;
        break;
     case 1:
        // Minutes to next track update
        waitMinutesToNextTrackUpdate=TempValue;
        break;
     case 2:
        // Set Samplerate on Volt, Amp, Temp
        VoltAmpSampleRate=TempValue;
        break;
     case 3:
        // Set time. Justerer kun på timerne
   //   setTime(18, 10, 10, 20, 7, 2015); //Ret tiden her, hvis det går helt galt.
        setTime(TempValue, minute(t), second(t), day(t), month(t), year(t)); 
        RTC.set(now());   //set the RTC from the system time
        break;
     case 4:
        // Set time. Justerer kun på minutterne
//      setTime(8, TempValue, 10, 15, 12, 2014); //Ret tiden her, hvis det går helt galt.
        setTime(hour(t), TempValue, second(t), day(t), month(t), year(t)); 
        RTC.set(now());   //set the RTC from the system time
        break;
     case 5:
        // Set differenceEastWest 
        SetdifferenceEastWest=TempValue;
        break;
     case 6:
        // Set differenceUpDown
        SetdifferenceUpDown=TempValue;
        break;
     case 7:
        // Powersave
        SleepTime=TempValue;
        break;
        
    }   
   ProgSeconds=-11; //Stop programmeringsløkken
   }
// midlertidig ude  NRF24L01_SendToPanel(2); // Send info til panelet om hvordan kontakterne står og om der er blevet ændret i parametrene.
}


// ************************************
// graphic commands to redraw the complete screen should be placed here  
// Selve kald proceduren til Hitachi displayet. Der skrives af een gang. 
// ************************************
void draw(void) 
{
//  Serial.println("Draw Menu");
  //  Sæt fonten f.eks  u8g2.setFont(u8g_font_unifont); // Default font
   u8g2.setFont(u8g_font_courB12); // eller med småt -  u8g2.setFont(u8g_font_timB10);

        //  switch drawString bestemmer hvilken tekst der skal skrives i displayet foroven
        switch (drawString) //Read the value of drawString
         {
         case 0:
               u8g2.drawStr( 1, 12, "-SolarPanel Tim Milgart-");
           break;
         case 1:
               u8g2.drawStr( 1, 12, "  Manuel. Going East    "); 
           break;
         case 2:
               u8g2.drawStr( 1, 12, "  Manuel. Going West    "); 
           break;
         case 3:
               u8g2.drawStr( 1, 12, "  Going home to East    "); 
           break;
         case 4:
               u8g2.drawStr( 1, 12, "Automatic. Going to East"); 
           break;
         case 5:
               u8g2.drawStr( 1, 12, "Automatic. Going to West"); 
           break;
         case 6:
               u8g2.drawStr( 1, 12, "Auto.  E/W in position  "); 
           break;
         case 7:
               u8g2.drawStr( 1, 12, " East END stop reached  "); 
           break;
         case 8:
               u8g2.drawStr( 1, 12, " West END stop reached  "); 
           break;
         case 9:
               u8g2.drawStr( 1, 12, "  Up END stop reached   "); 
           break;
         case 10:
               u8g2.drawStr( 1, 12, " Down END stop reached  "); 
           break;
         case 11:
               u8g2.drawStr( 1, 12, "   Manuel. Going Up     "); 
           break;
         case 12:
               u8g2.drawStr( 1, 12, "   Manuel. Going Down   "); 
           break;
         case 13:
               u8g2.drawStr( 1, 12, "  Automatic. Going Up   "); 
           break;
         case 14:
               u8g2.drawStr( 1, 12, " Automatic. Going Down  "); 
           break;
         case 15:
               u8g2.drawStr( 1, 12, "Auto.  U/D in position  "); 
           break;
         case 16:
               u8g2.drawStr( 1, 12, "   Shutdown. To windy   "); 
           break;
         case 17:
               u8g2.drawStr( 1, 12, "NO SUN. Powersave mode  "); 
           break;
         case 18:
               u8g2.drawStr( 1, 12, "West EndStop Overwrite !"); 
           break;
         case 19:
               u8g2.drawStr( 1, 12, "East EndStop Overwrite !"); 
           break;
         default:
               u8g2.drawStr( 1, 12, "-SolarPanel Tim Milgart-");

    //    case 18:     u8g2.drawStr( 45, 89, "Waiting for the sun at 6:00");   flyttet til showTrackupdate da den kun bruges der.

         }

   u8g2.drawFrame(1,16,239,60); //øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde 


   u8g2.drawStr( 4, 29, "Volt from Panel:      V ");
 char TmpVoltValue[6] =""; // en nye metode
   mappedVoltValue = Voltage*100;
   sprintf(TmpVoltValue, "%01d%01d.%01d%01d",(mappedVoltValue % 10000)/1000,(mappedVoltValue % 1000)/100,(mappedVoltValue % 100)/10,(mappedVoltValue % 10));
   u8g2.drawStr(172, 29,TmpVoltValue); // 172,30 er kun til volt måling pladsen



   u8g2.drawStr( 4, 44, "Amp  from Panel:      A ");
  char TempCurrentValue[6] ="";    // Den nye metode
   int NewCurrentValue = Amps*1000;
   sprintf(TempCurrentValue, "%01d.%01d%01d%01d",(NewCurrentValue % 10000)/1000,(NewCurrentValue % 1000)/100,(NewCurrentValue % 100)/10,(NewCurrentValue % 10));
   u8g2.drawStr(172, 44,TempCurrentValue); // 172,45 er kun til strøm måling pladsen



   u8g2.drawStr( 4, 59, "Temp from panel:      C "); // Viser temperaturen i panelet ved at aflæse temp sensor bag på panelet
  char TmpTempValue[6] ="";    // Den nye metode
    int mappedTempValue=Temperature*100;
    if (mappedTempValue < 0)
      { 
       mappedTempValue=abs(mappedTempValue); // så kan den ikke vise minus grader
       sprintf(TmpTempValue, "%01d%01d.%01d",(mappedTempValue % 10000)/1000,(mappedTempValue % 1000)/100,(mappedTempValue % 100)/10,(mappedTempValue % 10));
      } 
    else
        sprintf(TmpTempValue, "%01d%01d.%01d%01d",(mappedTempValue % 10000)/1000,(mappedTempValue % 1000)/100,(mappedTempValue % 100)/10,(mappedTempValue % 10));
    u8g2.drawStr(172, 59,TmpTempValue); // 172,30 er kun til temp måling pladsen



   u8g2.drawStr( 4, 74, "Total Watt/h   :     Wh "); //samlet antal KW produceret. Dvs der skal tæller på amp 
  char TmpString[5] = ""; // Den nye metode
  int sensor = TotalProducedEnergyHours; // viser ikke noget under 0,0, men først ved 1wh, da sensor er en integer og ikke float.
  sprintf(TmpString, "%04d",(sensor));  // så prøver vi med at læse energy over i integer variablen sensor 
  u8g2.drawStr(172, 74,TmpString);       // Skriv værdien i displayet


   // ramme til next track   
   u8g2.drawFrame(37,78,167,14); //øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde 


//   showTrackupdate() ;        // Viser resterende tid til trackupdate. Nedtællefunktion. 
  char NextTrackTimeAdjust[3] ="";
  char TrackTime[9] ="";  
  byte NextTrackUpdateinSeconds;
  time_t t = now();
if ((NextTrackUpdate-minute(t) <=1)||(SetNextTrackinSeconds==true))       // less than 1 min back and the flag is set to true
 {
   NextTrackUpdateinSeconds=60;      // make it to seconds by multiply with 60
   NextTrackUpdateinSeconds-second(t); // subtract the realtime seconds from the NextTrackUpdate, then we got the time back to NextTrackUpdate in seconds.
   SetNextTrackinSeconds=false;
 }

if (SetNextTrackinSeconds==false)      // We dont have to set the flag anymore, its already set 
 {   
   NextTrackUpdateinSeconds=NextTrackUpdateinSeconds-second(t); // subtract the realtime seconds from the NextTrackUpdate, then we got the time back to NextTrackUpdate in seconds.
 }

if  (NextTrackUpdateinSeconds==0)      //when NextTrackUpdateinSeconds hits 0, we start all over again
 {   
   SetNextTrackinSeconds=true;     
 }
 u8g2.setFont(u8g_font_ncenB08);
  if (drawString != 18)
       {
        u8g2.drawStr(40, 89, "Next track at: ");     
        if (NextTrackUpdate-minute(t) > 1) sprintf(TrackTime, "%02d:%02d:%02d",hour(t), NextTrackUpdate,0);  
        if (NextTrackUpdate-minute(t) <= 1) sprintf(TrackTime, "%02d:%02d:%02d",0 , 0,NextTrackUpdateinSeconds);  
        u8g2.drawStr(115,89, TrackTime);
        sprintf(NextTrackTimeAdjust, "%02d",waitMinutesToNextTrackUpdate);
        u8g2.drawStr(186,89, NextTrackTimeAdjust);
        // tiljøjer teksten til slut af feltet nexttrackupdate
        u8g2.setCursor(165,89);
        u8g2.print("adj:");
       }
  else
       {
         u8g2.drawStr( 45, 89, "Waiting for the sun at 6:00");                       
       } 

   
   // ramme til dato
   u8g2.drawFrame(37,94,167,34); //øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde 

   int lightSensorEastAverage = (lightSensorEastUpValue + lightSensorEastDownValue) / 2;       //Average East
   int lightSensorWestAverage = (lightSensorWestUpValue + lightSensorWestDownValue) / 2;       //Average West
   
   // Disse kan bruges til beregning af forskellig art ...senere 
   int lightSensorUpAverage   = (lightSensorEastUpValue + lightSensorWestUpValue) / 2;     //Average Top
   int lightSensorDownAverage = (lightSensorEastDownValue + lightSensorWestDownValue) / 2; //Average Down

   // Avarage fra alle sensorer, giver tal der kan måle om lyset er for svagt. 200= en gråvejrsdag i november uden sol og bare skyer.
   LightAverage = (lightSensorUpAverage + lightSensorDownAverage + lightSensorEastAverage + lightSensorWestAverage) / 4; 

   int DifVertikal = lightSensorUpAverage - lightSensorDownAverage;   // the difference between the 2 sunvalues Up and Down
   int DifHoizontal = lightSensorEastAverage - lightSensorWestAverage;// the difference between the 2 sunvalues East and West

   
   int differenceEastWest = abs(DifHoizontal);                             // 'abs' is the absolute value, the result is always positive.
   int differenceUpDown   = abs(DifVertikal);                              // 'abs' is the absolute value, the result is always positive.



   //ramme til East value
   u8g2.setFont(u8g_font_ncenB08);
   u8g2.drawFrame(1,78,35,14); //øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde 
//   u8g2.setPrintPos(4,89); 
//   u8g2.print("U");
   u8g2.drawStr( 4, 89, "U");
   u8g2.setCursor(12,89);            // Position til højre på skærmen
   u8g2.print(differenceUpDown); // viser hvad værdien af lightSensorEastValue


   //ramme til West value
   u8g2.drawFrame(205,78,35,14); //øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde 
//   u8g2.setPrintPos(207,89);
//   u8g2.print("W");
   u8g2.drawStr( 207, 89, "W");
   u8g2.setCursor(219,89);              // Position til venstre på skærmen
   u8g2.print(differenceEastWest);    // viser hvad værdien af lightSensorWestValue


   // Viser de reelle værdier for phototransistorerne.  Kun til test. Her kunne stå PowerNow i stedet for. Altså den værdi af watt der laves pt. Amp*Volt
   u8g2.setFont(u8g_font_courR08);
//   u8g2.setPrintPos(205,101);
//   u8g2.print("EU");
   u8g2.drawStr( 205, 101, "EU");
   u8g2.setCursor(222,101);
   u8g2.print(lightSensorEastUpValue);
   
//   u8g2.setPrintPos(205,110);
//   u8g2.print("WU");
   u8g2.drawStr( 205, 110, "WU");
   u8g2.setCursor(222,110);
   u8g2.print(lightSensorWestUpValue);

//   u8g2.setPrintPos(205,119);
//   u8g2.print("ED");
   u8g2.drawStr( 205, 119, "ED");
   u8g2.setCursor(222,119);
   u8g2.print(lightSensorEastDownValue);

//   u8g2.setPrintPos(205,128);
//   u8g2.print("WD");
   u8g2.drawStr( 205, 128, "WD");   
   u8g2.setCursor(222,128);
   u8g2.print(lightSensorWestDownValue);


   // halvcirkler til Sol meter   
   u8g2.drawDisc(17,128,11,U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT );

   // tegner en viser der bevæger sig i takt med ændring af variablen Amps
   int Sunconst = constrain(LightAverage, 500, 990); // limits range of LightAverage values to between 300 and 990
   int Sunline = map(Sunconst, 500, 990, 1, 40);     // LightAverage kunne også bruges her i stedet for Amps
   u8g2.drawLine(17, 128, Sunline, 105); 


   // små tal til Sol meter
   u8g2.setFont(u8g_font_baby); // små tal ved start og slut af meter
   u8g2.drawStr( 2, 128, "0");   
   u8g2.drawStr( 30, 128, "10");   


   // Tekst SUN til Sol meter
   u8g2.setFont(u8g_font_ncenB08);
   u8g2.drawStr( 6, 102, "SUN");
//   u8g2.setPrintPos(6,102);
//   u8g2.print("SUN");

   // Viser startdato og tid
//   showStartDateTime();
  char StartDate[20] ="";  // Den nye metode
  int elapseddays=0;
  int elapsedhours=0;
  int elapsedmins=0;
  long elapsedsecs=0;
  int intelapsedsecs=11;
  elapsedsecs = currentmillis/1000; //convect milliseconds to seconds
  elapsedmins=elapsedsecs/60; //convert seconds to minutes
  elapsedhours=elapsedmins/60; //convert minutes to hours
  elapseddays=elapsedhours/24; //convert hours to days
  elapsedsecs=elapsedsecs-(elapsedmins*60); //subtract the coverted seconds to minutes in order to display 59 secs max 
  elapsedmins=elapsedmins-(elapsedhours*60); //subtract the coverted minutes to hours in order to display 59 minutes max
  elapsedhours=elapsedhours-(elapseddays*24); //subtract the coverted hours to days in order to display 23 hours max
  u8g2.setFont(u8g_font_ncenB08);
  sprintf(StartDate, "%04d/%02d/%02d-%02d:%02d:%02d",startyear, startmonth, startday, starthour, startminute, startsecond);
  u8g2.drawStr(40, 115, "Start date :"); // Nederst i midten
  u8g2.drawStr(98, 115, StartDate); // skriver start dato
  intelapsedsecs=elapsedsecs;  // conv af float sekunder til int sekunder
  sprintf(StartDate, "%03d/%02d:%02d:%02d",elapseddays, elapsedhours, elapsedmins, intelapsedsecs); // forbrugt tid - Elapsed time
  u8g2.drawStr(40, 105, "Elapsed    :"); // Nederst i midten
  u8g2.drawStr(98, 105, StartDate); // skriver start tiden 



   if ( BadRTC==0 )
    { //showDateTime(); Viser dagsdato og tid.
      char DateTime[20] ="";  // Den nye metode
      time_t t = now();
      u8g2.setFont(u8g_font_ncenB08);
      u8g2.drawStr(40, 125, "Date          :"); // Nederst i midten
      sprintf(DateTime, "%04d/%02d/%02d-%02d:%02d:%02d",year(t), month(t), day(t),hour(t), minute(t), second(t));
      u8g2.drawStr(98, 125, DateTime); // 172,90 er kun til min:sek
    }
   else
     RTCerror(); //RTC blev ikke fundet
}

// ************************************
// ************************************
// This is the service menu. Where a bunch of parameters can be adjusted.
// Servicemenu -Service menu-
// ************************************
void Servicemenu1(void) 
{
//  Serial.println("Service Menu1");
  //  Sæt fonten f.eks  u8g2.setFont(u8g_font_unifont); // Default font
   u8g2.setFont(u8g_font_helvB08); // eller med småt -  u8g2.setFont(u8g_font_timB10);
   u8g2.drawFrame(0,11,239,78); //Ramme om menuen ... øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde 
     
      u8g2.drawStr( 1, 10,   "                      Service Menu 1              ");
      u8g2.drawStr( 5, 21,   "1. Service Menu 2                                 ");
      u8g2.drawStr( 5, 32,   "2. NextTrack update 1-15 min                      ");
      u8g2.drawStr( 5, 43,   "3. Samplerate for Volt/Amp 1-99                   ");
      u8g2.drawStr( 5, 54,   "4. Set Time, hour   0-23                          ");
      u8g2.drawStr( 5, 65,   "5. Set Time, minute 0-59                          ");
      u8g2.drawStr( 5, 76,   "6. Set differenceEastWest 0-99                    "); 
      u8g2.drawStr( 5, 87,   "7. Set differenceUpDown   0-99                    "); 

//     u8g2.drawStr( 5, 21,   "1. Sun sensor treshold 0-20 min                   ");
//      u8g2.drawStr( 5, 98,   "8.Powersave, Sleeptime  0-9 hours                ");
//      u8g2.drawStr( 5, 109,  "9.WiFi On/Off                                    ");
//      u8g2.drawStr( 5, 126,  "8. Exit"); 


   time_t t = now();      

   if (Prog == 0) // hvis ikke prog er trykket ind og programmering er i gang så
    {  
     // aflæs potmeter og lav værdien om til Menuvalg fra 1-9
     Menuvalg = analogRead(TolerancePot);  
     Menuvalg = map(Menuvalg, 0, 1023, 0, 9); // tilpasser y så den aldrig blir lavere en 0 eller højere end 9
     int y=12;
     y=y+(Menuvalg*11);
     if (Menuvalg !=9)
       u8g2.drawFrame(3,y,11,11); //Lille ramme om menu valget
     else
       u8g2.drawFrame(3,y+5,11,11); //øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde           
    }

   //Default værdierne skrives
   u8g2.setCursor(217,21);
   u8g2.print(":");
   u8g2.setCursor(222,21);
   u8g2.print(SensorThreshold);

   u8g2.setCursor(217,32);
   u8g2.print(":");
   u8g2.setCursor(222,32);
   u8g2.print(waitMinutesToNextTrackUpdate);

   u8g2.setCursor(217,43);
   u8g2.print(":");
   u8g2.setCursor(222,43);
   u8g2.print(VoltAmpSampleRate);

   u8g2.setCursor(217,54);
   u8g2.print(":");
   u8g2.setCursor(222,54);
   u8g2.print(hour(t));  //H for hours

   u8g2.setCursor(217,65);
   u8g2.print(":");
   u8g2.setCursor(222,65);
   u8g2.print(minute(t));  //M for minuts

   u8g2.setCursor(217,76);
   u8g2.print(":");
   u8g2.setCursor(222,76);
   u8g2.print(SetdifferenceEastWest);

   u8g2.setCursor(217,87);
   u8g2.print(":");
   u8g2.setCursor(222,87);
   u8g2.print(SetdifferenceUpDown);
/*
   u8g2.setCursor(217,98);
   u8g2.print(":");
   u8g2.setCursor(222,98);
   u8g2.print(SleepTime);

   u8g2.setCursor(217,109);
   u8g2.print(":");
   u8g2.setCursor(222,109);
   u8g2.print(WifiOn);
*/


  if ((Prog==1) && (ProgSeconds + 10) >= second(t))
   {  
    //indlæs i en temporær værdi som senere kan læses ind i den variabel der blir valgt.
    TempValue = analogRead(TolerancePot);  
    TempValue = abs(TempValue);              // 'abs' is the absolute value, the result is always positive.     
    u8g2.drawStr( 150,126,  "Old:");
    u8g2.drawStr( 190,126,  "New:");
    u8g2.drawFrame(218,116,15,12); //Ramme om den nye værdi øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde              
    u8g2.setCursor(170,126); //Viser nuværende værdi 
    
     //  switch valg bestemmer hvilken tekst der skal skrives i displayet forneden i menu feltet
    switch (Menuvalg) //Read the value of potmeter to chose submenu
    {
     case 0: //SensorThreshold
        ServiceMenu2 = true;
        break;
     case 1:// waitMinutesToNextTrackUpdate
        TempValue = map( TempValue, 0, 1020, 1, 15); // måler mellem 0 og 1023 i 8 bit til 0 og 15 min. Can't be less than 1.
        u8g2.print(waitMinutesToNextTrackUpdate);
        break;
     case 2: // VoltAmpSampleRate
        TempValue = map( TempValue, 0, 1020, 1, 99); // måler mellem 0 og 1023 i 8 bit til 1 og 99 samples. Can't be less than 1.
        u8g2.print(VoltAmpSampleRate);
        break;
     case 3: //Set time. Sætter timerne
        TempValue = map( TempValue, 0, 1020, 0, 23); // måler mellem 0 og 1023 i 8 bit til 0 og 23 timer
        u8g2.print(hour(t));
        break;
     case 4:  //Set time. Sætter minutterne
        TempValue = map( TempValue, 0, 1020, 0, 59); // måler mellem 0 og 1023 i 8 bit til 0 og 59 minutter
        u8g2.print(minute(t));
        break;
     case 5:  //Set differenceEastWest 0-99
        TempValue = map( TempValue, 0, 1020, 0, 99); // måler mellem 0 og 1023 i 8 bit til 0 og 99 
        u8g2.print(SetdifferenceEastWest);
        break;
     case 6:  //Set differenceUpDown 0-99
        TempValue = map( TempValue, 0, 1020, 0, 99); // måler mellem 0 og 1023 i 8 bit til 0 og 99
        u8g2.print(SetdifferenceUpDown);
        break;
/*     case 7: // SleepTime
        TempValue = map( TempValue, 0, 1023, 0, 9); // måler mellem 0 og 1023 i 8 bit til 0 og 9 
        u8g2.print(SleepTime);
        break;
     case 8: // Wifi On/Off
        TempValue = map( TempValue, 0, 1023, 0, 1); // måler mellem 0 og 1023 i 8 bit til 0 og 1. on eller off
        u8g2.print(WifiOn);     
        break;
*/     default:
        break;
    }
    u8g2.setCursor(220,126); 
    u8g2.print(TempValue);
   }
  else
   {
    Prog=0;
   } 


   // Viser de reelle værdier for phototransistorerne.  Kun til test.
   u8g2.drawFrame(0,92,205,12);   //Ramme om den nye værdi øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde              
//   u8g2.setFont(u8g_font_timR08); 
   u8g2.setCursor(3,102);
   u8g2.print("EU");
   u8g2.setCursor(18,102);
   u8g2.print(lightSensorEastUpValue);  

   u8g2.setCursor(43,102);
   u8g2.print("WU");
   u8g2.setCursor(62,102);
   u8g2.print(lightSensorWestUpValue);

   u8g2.setCursor(85,102);
   u8g2.print("ED");
   u8g2.setCursor(100,102);
   u8g2.print(lightSensorEastDownValue);

   u8g2.setCursor(122,102);
   u8g2.print("WD");
   u8g2.setCursor(140,102);
   u8g2.print(lightSensorWestDownValue);

   u8g2.setCursor(162,102);
   u8g2.print("AVR");
   u8g2.setCursor(186,102); // Disse 2 kan også bruges til noget nyttigt
   u8g2.print(LightAverage);  // LightAverage er en variable fra gennemsnittet at alt fra lyssensorerne




   // Viser antal RF errors
   u8g2.setCursor(3,115);
   u8g2.print("RF TotalErrors: ");
   u8g2.setCursor(82,115);
   u8g2.print(TotalError);

   // Viser batteri spændingen på det batteri der driver Mega2560
   u8g2.setCursor(3,126);
   u8g2.print("Battery: ");
   u8g2.setCursor(47,126);
   u8g2.print(Battery);

   // Skriver sekunder nederst på skærmen. Kan evt. fjernes.  
   u8g2.drawStr( 120,115,  "Seconds:");
   u8g2.setCursor(172,115);
   u8g2.print(second(t));
      



/*
   //Test af tolerance/sol sensitivity
   u8g2.setCursor(206,103);
   u8g2.print("Pot    ");
   
    if (ReadSensor == 1)  //skal den powerdown indtil den stiger over 20. lige nu er den 3238 på en gråvejrs dag...formeget. skal justeres 
    {
      u8g2.setCursor(230,103);
      u8g2.print("*");
    }  

   Tolerance = analogRead(TolerancePot); // read potentiometer and put it in Tolerance
   u8g2.setPrintPos(206,113); // Disse 2 kan også bruges til noget nyttigt
   u8g2.print(Tolerance);     // Tolerance er en variable fra potientiometeret 


   u8g2.setPrintPos(206,123); // Disse 2 kan også bruges til noget nyttigt
   u8g2.print(LightAverage);  // LightAverage er en variable fra gennemsnittet at alt fra lyssensorerne
   if (LightAverage > 50)    // viser hvornår der bliver målt ...over 50
   {
     u8g2.setPrintPos(230,123);
     u8g2.print("*");
   }  

*/

}



// ************************************
// Viser velkommen på skærmen mens den venter på wifi connect
// ************************************
void PanelWaitText()
{ 
   u8g2.setFont(u8g_font_courB10); // eller med småt -  u8g2.setFont(u8g_font_timB10);
   u8g2.drawStr( 1, 12, "Press button for Bigpanel");   
   u8g2.drawFrame(1,14,239,52); //øverste venstre hjørne X, øverste venstre hjørne Y, længde, bredde 
  if (BigpanelOn==3)
    ButtonProgState=digitalRead(ButtonProg);
   
  u8g2.setFont(u8g_font_courB12); // eller med småt -  u8g2.setFont(u8g_font_timB10);
   if (!ButtonProgState) // hvis udsagn er falskt, altså 0, så er prog trykket ind og MiniPanel er valgt
     {   
      BigpanelOn=1;
      u8g2.drawStr( 4, 30, "Comunicating with         ");
      u8g2.drawStr( 4, 45, "BigPanel                                         ");   
      u8g2.drawStr( 4, 60, "                                                 ");   
      
     } 
   else
     {
      BigpanelOn=0;
      u8g2.drawStr( 4, 30, "Comunication with         ");
      u8g2.drawStr( 4, 45, "MiniPanel                                        ");   
      u8g2.drawStr( 4, 60, "                                                 ");      
     }

   // Show version
   u8g2.drawStr( 1, 80, "Version 2.3.0           ");     

 
   if (BigpanelOn==1)   Serial.println("Panel is BIG");
   if (BigpanelOn==0)   Serial.println("Panel is MINI");
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
     Serial.print(second(t)); 

     prevsec=(second(t));  

     LoopCount = LoopCount +1;                   // counter for calculate
     totamps=totamps+Amps;                       // calculate total amps
     avgamps=totamps / LoopCount;                // average amps
     amphr = (avgamps*EnergyTime) / 3600;        // avg amp-hour
     ampsec = (avgamps*EnergyTime);        // avg amp-sekunder
     
     TotalProducedEnergyHours+(Voltage*amphr);    // akkumuleret energy i watttimer = Volt*amphr.  watt timer er baseret på en formodet opfyldning, men hvis den stopper i 1 min er det jo kun 59 minutter 
     
     Energysekunder=Voltage*ampsec;  // produceret energi i sekunder
     TotalProducedEnergysekunder=TotalProducedEnergysekunder+Energysekunder;

   }   
//////////////////////////////////////////////// current and energy calculation //////////////////////

    Serial.print("       Errors = "); Serial.print(Error);  
    Serial.print("       TotalErrors = ");Serial.print(TotalError);          

    TotalError+=Error;
    //Reset Error igen, så den er klar til næste fejl
    Error=0;
    
    Serial.print("       Antal loop gange "); Serial.print(counter_for_numbers_of_loops);   
    counter_for_numbers_of_loops++;
    
    // Så meget fri RAM er der tilbage
    Serial.print("   Free RAM: ");  Serial.println( freeMemory() );
    Serial.println(" ");    Serial.println(" ");
    
    prevsec=(second(t)); 

    // læser digitale værdier fra kontakterne til manuel motorstyring
    ReadButtons();                 

 if ((ButtonMotorMoveEastState == 1) && (ButtonMotorMoveWestState == 1)) // afbryder RF24 i af sende hvis kontakten står i midten. Så er jeg fri for at slukke 
    Serial.println(" ");
   else
{

   // Sender data til modtager        
   if (counter_for_numbers_of_loops % 2)  //motor info afsend hvert 1 sekund  & 2 = ODD
      NRF24L01_SendToPanel(2);  // 2= set motor og samplerate (crc=1309) Volt, Amp Temp, DrawString retur
   else
      NRF24L01_SendToPanel(3);  // 3= set motor og samplerate (crc=1280)og Lysværdi retur   
}
      
  Battery=analogRead(BatterySensor);     //read the voltage from the battery sensor
  Battery = map( Battery, 0, 1023, 0, 2500 ); // måler mellem 0 og 24 volt
  Battery=Battery/100; // da map er i tusinder
  if (Battery < 7.2) //11.7 el 7.95      10.7 er til bilbatteri      7.2 er til 2 Cell Lipo internt
    digitalWrite(BLUE, LOW);   // Tænder dioden
  else
    digitalWrite(BLUE, HIGH);  // Slukker igen
  Serial.print("Battery Volt = ");    Serial.println(Battery);  


  if (TotalError > 1)  // 0 is to low
    digitalWrite(RED, LOW);  //rød tænder
  else
    digitalWrite(RED, HIGH); //rød slukker

  digitalWrite(GREEN, HIGH);  //grøn, high slukker den 


  //Radio lukkes ned
  //radio.powerDown(); 

  // picture loop
  u8g2.firstPage();  
  do {
    if ( servicemenu ==0) 
      draw();  // show display
    else
      {
        if (ServiceMenu2 == false) // Hvis servicemenu 2 er valgt så hop til den ellers forsæt
            Servicemenu1(); // service menu 1  
//        else
//            Servicemenu2(); // service menu 2 
      }
     } while( u8g2.nextPage() );



   
  // Radio startes op
  //radio.powerUp();
  //delay(5);    

  
  // nulstiller drawString så den er klar til nye actions der skal op på skærmen.  
  drawString = 0; 
  digitalWrite(RED, HIGH); //rød slukker....dvs den står og blinker når der har været afbrudt til radio link.  "no radio link"
}
