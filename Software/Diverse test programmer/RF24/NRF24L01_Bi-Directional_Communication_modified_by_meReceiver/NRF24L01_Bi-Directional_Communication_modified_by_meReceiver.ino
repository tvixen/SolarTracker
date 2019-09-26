/*
 *
 * NRF24L01_Bi-Directional_Communication
 *
 * Testbed for NRF24L01 boards
 *
 * Bi-directional traffic
 * https://www.youtube.com/watch?v=a2FBa4MK66M
 * This sketch uses the same software for both nodes
 *
 * written by Andreas Spiess. Based on Example Sketch of RF24 Library
 * 
 * 
 * 
 * THIS IS FOR THE DiSPLAY
 * 
 * 

 */

#include <SPI.h>
#include <RF24.h>
#include <Adafruit_NeoPixel.h>
#include <printf.h>


// Colors read by TCS3200
int red = 0;
int green = 0;
int blue = 0;



#define COUNT 10 // nomber for statistics

#define DATARATE RF24_2MBPS
//  #define DATARATE RF24_1MBPS
// #define DATARATE RF24_250KBPS

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xDEDEDEDEE1LL, 0xDEDEDEDEE2LL }; // WritingPipe, ReadingPipe For SmallPanel

RF24 radio(45, 49);


int _success = 0;
int _fail = 0;
unsigned long _startTime = 0;




typedef struct  //Incomming
{

  int lightSensorEastValue;
  int lightSensorWestValue;
  int NextTrackOffsetANDwaitMinutesToNextTrackUpdate;

 }
PanelDef;
PanelDef SensorPanelPak;



typedef struct //Outgoing
{

  bool ButtonMotoEWOnState;
  bool ButtonMotorMoveEastState;
  int waitMinutesToNextTrackUpdate;
  byte VoltAmpSampleRate;
  int SetdifferenceEastWest;
  int timer;
  int minutter;
  int sekunder;
  int dag;
  int maaned;
  int aar;

 }
displayDef;
displayDef displayPak;


void setup(void)
{
  Serial.begin(115200);
  printf_begin();
  printf("\n\rRF24/examples/pingpair/\n\r");
  printf("ROLE: Receiver");

  // Setup and configure RF radio
  radio.begin();
  radio.openWritingPipe(pipes [1]);
  radio.openReadingPipe(1, pipes[0]);
  radio.setDataRate( DATARATE ) ;
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.setChannel(2);
  radio.enableDynamicPayloads() ;
  radio.enableAckPayload();               // not used here
  radio.setRetries(0, 15);                // Smallest time between retries, max no. of retries
  radio.setAutoAck( true ) ;
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  radio.powerUp();
  radio.startListening();
} 

void loop(void)
{
   unsigned long loop_start = millis();

 // if there is data ready

  while ( !radio.available() && (millis() - loop_start) < 200) {
      // Serial.println(F("waiting."));
    }
    if (millis() - loop_start >= 200) {
      // printf("Failed. Timeout: %i...", millis() - loop_start);
      _fail++;
    } else {
      // get the telemetry data
      radio.read(&SensorPanelPak, sizeof(SensorPanelPak));
     // Serial.print("Got response ");
      _success++;
    }


  

/*   Received by display
 
    if ( radio.available())
    {
      radio.read(&controlPak, sizeof(controlPak));

*/
 int _ratio = 100 * _fail / (_fail + _success);
      Serial.print("Time ");
      _startTime = (millis() - _startTime);
      Serial.print(_startTime);
      Serial.print(" success ");
      Serial.print(_success);
      Serial.print(" timeout ");
      Serial.print(_fail);
      Serial.print(" Failed ");
      Serial.print(_ratio);
      Serial.println("% ");

/*
     
      displayPak.red = (float)red;
      displayPak.green = (float)green;
      displayPak.blue = (float)blue;
*/

  
      Serial.print(" Received lightSensorEastValue ");
      Serial.print(SensorPanelPak.lightSensorEastValue);
      Serial.print(" Received  lightSensorWestValue ");
      Serial.print(SensorPanelPak.lightSensorWestValue);
      Serial.print(" Received  NextTrackOffsetANDwaitMinutesToNextTrackUpdate ");
      Serial.print(SensorPanelPak.NextTrackOffsetANDwaitMinutesToNextTrackUpdate);
      Serial.println();
/*  
      Serial.print(" red ");
      Serial.print(displayPak.red);
      Serial.print(" green ");
      Serial.print(displayPak.green);
      Serial.print(" blue ");
      Serial.print(displayPak.blue);
      Serial.println();
*/
      // Send the final one back. This way, we don't delay
      // the reply while we wait on serial i/o.
      radio.stopListening();
      radio.write( &displayPak, sizeof(displayPak) );
      // Serial.print("Sent response ");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
      Serial.println("Indhold: "); 
      Serial.println(SensorPanelPak.lightSensorEastValue);
      Serial.println(SensorPanelPak.lightSensorWestValue);
      Serial.println(SensorPanelPak.NextTrackOffsetANDwaitMinutesToNextTrackUpdate); 
      Serial.println();
      _success = 0;
      _fail = 0;
      _startTime = millis();
      
  // } //if radio.avalible
}


