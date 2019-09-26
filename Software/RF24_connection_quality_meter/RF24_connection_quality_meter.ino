/*
 PROJECT: MySensors / Quality of radio transmission 
 PROGRAMMER: AWI (MySensors libraries)
 DATE: 20160529/ last update: 20160530
 FILE: AWI_Send.ino
 LICENSE: Public domain

 Hardware: ATMega328p board w/ NRF24l01
  and MySensors 2.0
  
Special: https://forum.mysensors.org/topic/3984/nrf24l01-connection-quality-meter
  
  
Summary:
  Test the RF24 quality and range.
  Sends a radio message with counter each  x time to determine fault ratio with receiver.
  
  The sketch sends values from 0..99 to a gateway with "ack" enabled. The LCD shows the "failed" 
  sends and the number of not acknowledged messages. i.e. the messages which did not arrive at 
  the gateway or did not make it on their return flight.
  
Remarks:
  Fixed node-id & communication channel to other fixed node
  
Change log:
20160530 - added moving average on fail/ miss count, update to 2.0
05122018 - Removed I2C LCD and added parallel, as this is what i have in the drawer.

*/


//****  MySensors *****
// Enable debug prints to serial monitor
#define MY_DEBUG 
#define MY_RADIO_NRF24                  // Enable and select radio type attached
//#define MY_RF24_CHANNEL 80                // radio channel, default = 76

#define MY_NODE_ID 250
#define NODE_TXT "Q 250"                // Text to add to sensor name

//#define MY_PARENT_NODE_ID 42              // fixed parent to controller when 0 (else comment out = AUTO)


// #define MY_RF24_CE_PIN 7               // Ceech board, 3.3v (7,8)  (pin default 9,10)
// #define MY_RF24_CS_PIN 8
#define DESTINATION_NODE 0                // receiving fixed node id (default 0 = gateway)

#include <SPI.h>
#include <MySensors.h>  

// display
#include <LiquidCrystal.h>                // LCD display with I2C interface


// helpers
#define LOCAL_DEBUG

#ifdef LOCAL_DEBUG
#define Sprint(a) (Serial.print(a))           // macro as substitute for print, enable if no print wanted
#define Sprintln(a) (Serial.println(a))         // macro as substitute for println
#else
#define Sprint(a)                   // enable if no print wanted -or- 
#define Sprintln(a)                   // enable if no print wanted
#endif


// MySensors sensor
#define counterChild 0

// send constants and variables
int messageCounter = 0 ; 
const int messageCounterMax = 100 ;           // maximum message counter value 
const unsigned counterUpdateDelay = 500 ;       // send every x ms and sleep in between

// receive constants and variables
boolean failStore[messageCounterMax] ;          // moving average stores & pointers
int failStorePointer = 0 ;
boolean missedStore[messageCounterMax] ;
int missedStorePointer = 0 ;
int newMessage = 0 ;
int lastMessage = -1 ;
int missedMessageCounter = 0 ;              // total number of messages in range (messageCounterMax)
int failMessageCounter = 0 ;              // total number of messages in range (messageCounterMax)
uint8_t parent = 0 ;                  // parent node-id 

// Loop delays
const unsigned long displayInterval = 1000UL ;      // display update in ms
unsigned long lastDisplayUpdate = 0 ;         // last update for loop timers

// standard messages
MyMessage counterMsg(counterChild, V_PERCENTAGE);   // Send value

// ***** LCD
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
//LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD I2C address

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {

    // ** LCD display **
    // LCD 2 lines * 16 char.
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
  lcd.print("AWI Quality nRF24");

  for(int i= 0 ; i <  messageCounterMax ; i++){   // init stores for moving averages
    failStore[i] = true ;
    missedStore[i] = true ;
  }
  missedStorePointer = failStorePointer = 0 ;
  delay(1000);
}

void presentation(){
// MySensors
  present(counterChild, S_DIMMER, "Quality counter " NODE_TXT) ;  // counter uses percentage from dimmer value
}


void loop() {
  // Sprint("count:") ; Sprintln(messageCounter) ;
  LCD_local_display();
  missedStore[failStorePointer] = false  ;      // set slot to false (ack message needs to set) ; 
  boolean succes = failStore[failStorePointer] = send(counterMsg.setDestination(DESTINATION_NODE).set(failStorePointer), true);  // send to destination with ack
  if (!succes){
    failMessageCounter++ ; 
    Sprint("Fail on message: ") ; Sprint(failStorePointer) ;
    Sprint(" # ") ; Sprintln(failMessageCounter);
  }
  failStorePointer++ ;
  if(failStorePointer >= messageCounterMax){
    failStorePointer =  0 ;           // wrap counter
  }
  parent = getParentNodeId();             // get the parent node (0 = gateway)
  wait(counterUpdateDelay) ;              // wait for things to settle and ack's to arrive
}

void receive(const MyMessage &message) {          // Expect few types of messages from controller
  newMessage = message.getInt();            // get received value
  switch (message.type){
    case V_PERCENTAGE:
      missedStore[newMessage] = true ;      // set corresponding flag to received.
      if (newMessage > lastMessage){        // number of messages missed from lastMessage (kind of, faulty at wrap)
        Sprint("Missed messages: ") ; Sprintln( newMessage - lastMessage - 1) ;
        missedMessageCounter += newMessage - lastMessage - 1 ;
      }
      lastMessage = newMessage ;
      break ;
    default: break ;
  }
}


// calculate number of false values in array 
// takes a lot of time, but who cares...
int getCount(boolean countArray[], int size){
  int falseCount = 0 ;
  for (int i = 0 ; i < size ; i++){
    falseCount += countArray[i]?0:1 ;
  }
  return falseCount ;
}

void LCD_local_display(void){
/* prints all available variables on LCD display with units
*/
  
    char buf[17];                       // buffer for max 16 char display
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof buf, "p%-3dFail%4d%3d%%", parent, failMessageCounter, getCount(failStore, messageCounterMax));
    lcd.print(buf);
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof buf, "d%-3dMiss%4d%3d%%", DESTINATION_NODE , missedMessageCounter, getCount(missedStore, messageCounterMax));
  lcd.print(buf);
}

