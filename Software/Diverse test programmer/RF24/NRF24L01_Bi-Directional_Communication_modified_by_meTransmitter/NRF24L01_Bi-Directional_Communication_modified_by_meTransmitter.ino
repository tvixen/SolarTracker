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

 */

#include <SPI.h>
#include <RF24.h>
#include <printf.h>

#define DATARATE RF24_2MBPS
//  #define DATARATE RF24_1MBPS
// #define DATARATE RF24_250KBPS

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xDEDEDEDEE1LL, 0xDEDEDEDEE2LL };  //Write, Read
RF24 radio(45, 49);


int _success = 0;
int _fail = 0;
unsigned long _startTime = 0;


typedef struct //Outgoing
{
  int A0;
  int A1;
  int A2;
}
controlDef;
controlDef controlPak;


typedef struct //Incomming
{
  float red;
  float green;
  float blue;
}
displayDef;
displayDef displayPak;

void setup(void)
{
  Serial.begin(115200);


  printf_begin();
  printf("\n\rRF24/examples/pingpair/\n\r");
  printf("ROLE: Transmitter");

  // Setup and configure RF radio
  radio.begin();
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);
  radio.setDataRate( DATARATE ) ;
  radio.setPALevel( RF24_PA_MIN) ;
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

    // Take the time, and send it.  This will block until complete
   // pinMode(A0, INPUT);
   //   pinMode(A1, INPUT);
   // pinMode(A2, INPUT);

    controlPak.A0 = 1000;   // red
    controlPak.A1 = 1001;   // green
    controlPak.A2 = 1002;   // blue
 
    //controlPak.A0 = analogRead(A0);   // red
    //controlPak.A1 = analogRead(A1);   // green
    
    //Serial.print("A1 =");Serial.println(controlPak.A1);
    //controlPak.A2 = analogRead(A2);   // blue

    // printf("Now sen ding %i...", controlPak.steering);

    // First, stop listening so we can talk.
    radio.stopListening();
    if (!radio.write( &controlPak, sizeof(controlPak) )) {
      // Serial.println(F("failed."));
    }
    radio.startListening();
    // Serial.println(F("delivery success."));
    // printf("Time: %i ", millis() - loop_start);

    while ( !radio.available() && (millis() - loop_start) < 200) {
      // Serial.println(F("waiting."));
    }
    if (millis() - loop_start >= 200) {
      // printf("Failed. Timeout: %i...", millis() - loop_start);
      _fail++;
    } else {
      // get the telemetry data
      radio.read( &displayPak, sizeof(displayPak) );
     // Serial.print("Got response ");
      _success++;
    }

    if (_fail + _success >= 10)
    {
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
      Serial.print("% ");
      for (int _i = 0; _i < _ratio; _i++) Serial.print("*");
      Serial.print(" red =");
      Serial.print(displayPak.red);
      Serial.print(" GREEN ="); 
      Serial.print(controlPak.A1);
      Serial.print(" green =");
      Serial.print(displayPak.green);
      Serial.print(" blue =");
      Serial.print(displayPak.blue);
      Serial.println();
      _success = 0;
      _fail = 0;
      _startTime = millis();
    }

}


