/*
 * 
 * 24/11-2018
 * Denne kode virker er til test printet hvor der er monteret en blå diode samt OLE og ADS1115.
 * Til små forsøgsopstillinger.
 * Her en strøm opstilling med shunt på 20A til 75mv
 *  
 */


#include <avr/wdt.h>     // watchdog
#include <Wire.h>        // kan tale med Alle Arduino print
#include <Adafruit_ADS1015.h>    // ADC 1115 
/*
 * ADC addresses:
 * 0x4A = SDA  No 1 / id 24
 * 0x49 = 5V   No 2 / id 25
 * 0x48 = GND  No 3 / id 26
 * 0x4B = SLC  
 */
Adafruit_ADS1115 ads1115(0x48);  // construct an ads1115 at address 

#include "SSD1306Ascii.h"        // Font for Oled
#include "SSD1306AsciiWire.h"    // Font of Wire for Oled
// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C        // Oled address
/*
 * OLED addresses:
 * 0x3D = No 1 / id 24
 * 0x3C = No 2 / id 25
 * 0x3C = No 3 / id 26
 */
// Define proper RST_PIN if required.
#define RST_PIN -1              // Reset pin for Oled
SSD1306AsciiWire oled;          // Construct of AsciiWire 

float ADCvalue = 0;


void setup() {
  Wire.setTimeout(1000);
  Wire.setClock(100000);  //100khz
  Wire.begin();    
  Serial.begin(115200);


  //3 x blink 
  ledGreen();  //tænder grønled
  delay(200);
  ledGreen();  //tænder grønled
  delay(200);
  ledGreen();  //tænder grønled
  delay(200);
  ledGreen();  //tænder grønled
  delay(200);
  ledGreen(); 

 

//  initTimer1(); // Timer til interrupt af ADC
  initOled();      // Oled display startup
  
  initADC();      // kald ads1115.begin();
//  ReadTemp();     // læs temperatur
  ReadADC();      // læs spænding
//  PackNumber();    // Batteri pakken får nummer i displayet
//  drawVoltage();   // skriv ud på OLED
//  Check_if_Overload(); //Kontrol af spændingen på batteriet  
   Serial.println(" Setup done version 10_25");
//  init_i2c();
 // watchdogArm();   //sætter vagthunden til
}
//----------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------


void loop() {
  wdt_reset();     // Watchdog reset
 // ReadTemp();
  ReadADC();
  drawVoltage();   // læs ADCvalue 
}

//------------------------------------------------------------------------------------------------------------
bool x;
inline void ledGreen() //tænder eller slukker LED
{
if(x)
 {
  // High  2 byte sparet ved at skrive asm kode i forholdt til digitalwrite
  DDRB |= (1 << DDB1);
  PORTB |=  (1 << PB5);    //pin 6 på Attiny85, på min D13 PB5.  Så det kan måske virke.
 }
else
 {
  DDRB |= (1 << DDB1);
  PORTB &= ~(1 << PB5);
 }

x=!x;
  
}

inline void ledOff() 
{
  //Low  2 byte sparet ved at skrive asm kode i forholdt til digitalwrite
  DDRB |= (1 << DDB1);
  PORTB &= ~(1 << PB5);
}
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
void initADC()
{
  //                                       Max volt:         
  // ads1115.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads1115.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads1115.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads1115.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads1115.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads1115.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV 
  
   ads1115.setGain(GAIN_SIXTEEN);     // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
   ads1115.begin();  // start of ADC ADS1115
}
//------------------------------------------------------------------------------------------------------------
void ReadADC()
{
    
 //float ADCvalue = 0;
 uint16_t results    = 0;
 float AmpsPrMillivolt = 0.04172F;  //75mv pr 50 amp. 50/75= 0,666mv pr. 1 Amp
 float multiplier = 0.125F; /*mv pr counts 0,125/2048 = (16-bit results) 0.125 */
/*
  for(int i=0;i<10;i++)  // taking 10 samples from sensors with a inerval of 2msec and then average the samples data collected
    {
      results = ads1115.readADC_Differential_0_1();  
      delay(2);
    }
  results=results/10;         //Hiv gennemsnittet af de 10 målinger
*/


  results = ads1115.readADC_Differential_0_1();  
//  if (results<1) results = 0;
//  if (results>6000) results = 0;  
  Serial.print("Differential: "); Serial.print(results); 
 
  ADCvalue=results*multiplier;   
  Serial.print("  Meassured mv: "); Serial.print(ADCvalue); Serial.print("mv");

  ADCvalue=ADCvalue*AmpsPrMillivolt; 
  Serial.print("  Meassured Amps: "); Serial.print(ADCvalue); Serial.println("Amps");
  
  /*
 results = ads1115.readADC_SingleEnded(0);
  Serial.print("SingleEnded A0: "); Serial.println(results); 
  delay(150);
  results = ads1115.readADC_SingleEnded(1);
  Serial.print("SingleEnded A1: "); Serial.println(results); 
  delay(150);
  results = ads1115.readADC_SingleEnded(2);
  Serial.print("SingleEnded A2: "); Serial.println(results); 
  delay(150);
  results = ads1115.readADC_SingleEnded(3);
  Serial.print("SingleEnded A3: "); Serial.println(results); 
*/

  

  delay(500);
 //  Serial.println(" ");
  
}
//------------------------------------------------------------------------------------------------------------

void initOled()
{
  
#if RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
#else // RST_PIN >= 0
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
#endif // RST_PIN >= 0

  oled.setFont(Verdana12);
  oled.set1X();
  oled.clear();
  oled.setCursor(0,0);
  oled.setLetterSpacing(1);
  oled.print("Made By Tim Milgart");  
  delay(1000);
  oled.clear();  
  
}

void drawVoltage(void)
{
  oled.set2X();
  oled.setFont(lcdnums14x24);
  oled.setCursor(0,0);
  Serial.print("ADC Value ="); Serial.println(ADCvalue); 
  oled.print(ADCvalue,2);        // viser hele tallet 4½ cifre..overvejer at sløjfe det sidste.

}


//------------------------------------------------------------------------------------------------------------
