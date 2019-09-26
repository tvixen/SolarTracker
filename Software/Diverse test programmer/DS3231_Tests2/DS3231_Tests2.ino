/*
 * It programs an alarm on the DS3231 clock at a given time every day. 
 * Change the time for your test, compile the sketch and load it. The sketch is fully commented to explain what is done.
 * The test uses the on-board led on Arduino for evidencing the activity status.
 * When the sketch starts, the led should lit up for 1 second, to signal that it started correctly, than Arduino should enter the sleep mode: the led is set to OFF for indicating this.
 * At the programmed time, the DS3231 will produce an interrupt on the SQW pin connected to the Arduino D2 pin (which goes from the HIGH to the LOW level) 
 * which will wake up the Arduino from the sleep state, and the led should start to blink to indicate this.
 * Taken from the page: https://www.instructables.com/id/Arduino-Sleep-and-Wakeup-Test-With-DS3231-RTC/
 */

#include <Wire.h>
#include <RTClibExtended.h>
#include <LowPower.h>


#define wakePin 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define ledPin 13    //use arduino on-board led for indicating sleep or wakeup status

RTC_DS3231 RTC;      //we are using the DS3231 RTC EXTENDED

byte AlarmFlag = 0;
byte ledStatus = 1;
byte Port2 = 0;
//-------------------------------------------------

void wakeUp()        // here the interrupt is handled after wakeup
{

  Serial.println("Now its time to Wakeup");
 
}

//------------------------------------------------------------

void setup () {


  Serial.begin(115200);

  delay(300); // wait for console opening

  if (! RTC.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (RTC.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
   //  RTC.adjust(DateTime(2018, 6, 3, 16, 16, 20));
  }

 //  RTC.adjust(DateTime(2018, 6, 3, 16, 23, 30));
  
  
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(wakePin, INPUT);

  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(100);

  //Initialize communication with the clock
  Wire.begin();
  RTC.begin();
 // RTC.adjust(DateTime(__DATE__, __TIME__));   //set RTC date and time to COMPILE time
  
  //clear any pending alarms
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  RTC.armAlarm(2, false);
  RTC.clearAlarm(2);
  RTC.alarmInterrupt(2, false);

  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
  //The output of the DS3231 INT pin is connected to this pin
  //It must be connected to arduino D2 pin for wake-up
  RTC.writeSqwPinMode(DS3231_OFF);

  //Set alarm1 every day at 18:33
  //RTC.setAlarm(ALM1_MATCH_SECONDS, 15, 29, 10);  //Virker på 10 sekunder 
  //RTC.setAlarm(ALM1_MATCH_HOURS, 15, 59, 10);   //virker ikke som tt:mm:ss set your wake-up time here, match hours *and* minutes, seconds
  //RTC.setAlarm(ALM1_MATCH_HOURS, 10, 31, 15);   //virker ikke som tt:mm:ss set your wake-up time here, match hours *and* minutes, seconds
  //RTC.setAlarm(ALM1_MATCH_HOURS, 13, 15, 40, 01);   //virker ikke som tt:mm:ss set your wake-up time here, match hours *and* minutes, seconds
  //RTC.setAlarm(ALM1_MATCH_HOURS, 12, 16, 4);   //virker som mm:tt:ss set your wake-up time here, match hours *and* minutes, seconds
  //RTC.setAlarm(ALM1_MATCH_HOURS, 30, 16, 40);  //de 40 blir til 0
  //RTC.setAlarm(ALM1_MATCH_HOURS, 30, 16, 40, 3);  //virker ikke
  //RTC.setAlarm(ALM1_MATCH_HOURS, 39, 16, 0);  //virker  som 16:39:0
  RTC.setAlarm(ALM1_MATCH_HOURS, 39, 16, 0);  //virker  som 16:39:0.  så sekunder virker ikke
  RTC.alarmInterrupt(1, true);
}

//------------------------------------------------------------
void printtime() 
{
    DateTime now = RTC.now();
     Serial.println("i kaldet printime ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    Port2 = digitalRead(2); Serial.print("Pin2 efter sleep3 = "); Serial.println(Port2);

    Serial.println();
    
    Serial.println();

}


//------------------------------------------------------------

void loop() {

   printtime();
DateTime now = RTC.now();
  //On first loop we enter the sleep mode
//if (now.minute()== 33)
   if (AlarmFlag == 0) 
  {
    attachInterrupt(0, wakeUp, LOW);                       //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW 
    digitalWrite(ledPin, LOW);                             //switch-off the led for indicating that we enter the sleep mode
    ledStatus = 0;                                         //set the led status accordingly
    Serial.println("Time to Sleep until seconds is 10");
    delay(500);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);   //arduino enters sleep mode here
 //   Port2 = digitalRead(2); Serial.print("Pin2 efter sleep 1 = "); Serial.println(Port2);
    detachInterrupt(0);                                    //execution resumes from here after wake-up

    //When exiting the sleep mode we clear the alarm
    RTC.armAlarm(1, false);
    RTC.clearAlarm(1);
    RTC.alarmInterrupt(1, false);
    AlarmFlag++;
  }

  //cycles the led to indicate that we are no more in sleep mode
  if (ledStatus == 0) {
    ledStatus = 1;
    digitalWrite(ledPin, HIGH);
  }
  else {
    ledStatus = 0;
    digitalWrite(ledPin, LOW);
  }

  
  delay(500);

  Serial.println();
  Serial.println();
    
}

