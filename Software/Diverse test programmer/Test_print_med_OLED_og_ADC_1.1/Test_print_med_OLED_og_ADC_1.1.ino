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
Adafruit_ADS1115 ads1115CurrentModule(0x48);  // GND construct an ads1115 at address 

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

//----------------------------------------------------------------------------------------------------------------------
void setup() {
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

  initOled();      // Oled display startup
  initADC();      // kald ads1115.begin();
  Serial.println(" Setup done version Test_print_med_OLED_og_ADC_1.1");
}
//----------------------------------------------------------------------------------------------------------------------


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
  
   ads1115CurrentModule.setGain(GAIN_SIXTEEN);     // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
   ads1115CurrentModule.begin();  // start of ADC ADS1115
   ads1115CurrentModule.setSPS(ADS1115_DR_860SPS);
}
//------------------------------------------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//How to measure AC current with the DL-CT08CL5-20A/10mA 2000/1 0~120A Micro Current Transformer HH.                                            //           
//                                                                                                                                              // 
//Make a little circuit of 3 resistors and a cap(10uf) + the coil.                                                                              //
//Size of R3 is  primary current divided with the turn ratio.  Mine is 20A 2000T:  20/2000=0,01 or 0,01mA.                                      // 
//Reference from R1 and R2 (both 10K) gives 2.5volt, so 2.5/0,01= 250ohm. So the burden resistor should be 250ohm, but I should measure it!!!!  //  
// drawing: https://www.the-diy-life.com/simple-arduino-home-energy-meter/                                                                      // 
// look also here: https://learn.openenergymonitor.org/electricity-monitoring/voltage-sensing/measuring-voltage-with-an-acac-power-adapter      //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReadAC_from_ADC()
{ 
  int currentPin = 1;              //Assign CT input to pin 1
  double kilos = 0;
  int peakPower = 0;
  #define LysnetAC 230

  
  
  int current = 0;
  int maxCurrent = 0;
  int minCurrent = 1000;
  
  unsigned long startMillis;
  unsigned long endMillis;

  for (int i=0 ; i<=20 ; i++)  //Monitors and logs the current input for 200 cycles to determine max and min current
  {
    current = ads1115CurrentModule.readADC_Differential_0_1();
    //current = analogRead(currentPin);    //Reads current input and records maximum and minimum current
    if(current >= maxCurrent)
      maxCurrent = current;
    else if(current <= minCurrent)
      minCurrent = current;
      delay(20);
  }
  if (maxCurrent <= 517)
  {
    maxCurrent = 516;
  }
  
  double RMSCurrent = ((maxCurrent - 516)*0.707)/11.8337;    //Calculates RMS current based on maximum value. Adjust 11.8337 to be more precise. do it with a known bulb (40w)
  int RMSPower = LysnetAC*RMSCurrent;    //Calculates RMS Power Assuming Voltage 230VAC, change to 220VAC accordingly
  if (RMSPower > peakPower)
  {
    peakPower = RMSPower;
  }

   
  endMillis = millis();
  unsigned long time = endMillis - startMillis;
  kilos = kilos + (RMSPower * (time/60/60/1000000));    //Calculate kilowatt hours used
  startMillis = millis();

  Serial.print("Differential: "); Serial.println(current); 
  Serial.print("maxCurrent: "); Serial.println(maxCurrent); 
  Serial.print("minCurrent: "); Serial.println(minCurrent); 
  Serial.print("RMSCurrent: "); Serial.println(RMSCurrent); 
  Serial.print("startMillis: "); Serial.println(startMillis); 
  Serial.print("endMillis: "); Serial.println(endMillis); 
  Serial.print("time: "); Serial.println(time); 
  Serial.print(RMSCurrent);
  Serial.println("A");
  Serial.print(RMSPower);
  Serial.println("W");
  Serial.print(kilos);
  Serial.println("kWh");
  Serial.print(peakPower);
  Serial.println("W");
}
//------------------------------------------------------------------------------------------------------------
void ReadDC_from_ADC()
{
    
 //float ADCvalue = 0;
 uint16_t results,SampResults    = 0;
 uint16_t Read_BatteryPowerCurrent       = 0;  // To current Shunt 20A = 75mv 
 float AmpsPrMillivolt = 0.277F;  //75mv pr 50 amp. 50/75= 0,666mv pr. 1 Amp
 float multiplier = 0.125F; /*mv pr counts 0,125/2048 = (16-bit results) 0.125 */

 multiplier = ads1115CurrentModule.voltsPerBit()*1000.0F;    /* Sets the millivolts per bit */

  for(int i=0;i<20;i++)  // taking 10 samples from sensors with a inerval of 2msec and then average the samples data collected
    {
      SampResults += ads1115CurrentModule.readADC_Differential_0_1();  
      delay(10);
    }
  SampResults=SampResults/20;         //Hiv gennemsnittet af de 10 målinger

  Serial.print("Differential1: "); Serial.print(SampResults); 
  ADCvalue=SampResults*multiplier;   
  Serial.print("  Meassured mv: "); Serial.print(ADCvalue); Serial.print("mv");
  ADCvalue=ADCvalue*AmpsPrMillivolt; 
  Serial.print(" Sampled Meassured Amps: "); Serial.print(ADCvalue); Serial.println("Amps");

  delay(200);

 Read_BatteryPowerCurrent = ADCvalue*10; 
  Serial.print("Result from A0-A1: "); Serial.println(Read_BatteryPowerCurrent);
  Serial.println(" ");
/*
  results = ads1115CurrentModule.readADC_Differential_0_1();  
//  if (results<1) results = 0;
//  if (results>6000) results = 0;  
  Serial.print("Differential2: "); Serial.print(results); 
 
  ADCvalue=results*multiplier;   
  Serial.print("  Meassured mv: "); Serial.print(ADCvalue); Serial.print("mv");

  ADCvalue=ADCvalue*AmpsPrMillivolt; 
  Serial.print("  Single Meassured Amps: "); Serial.print(ADCvalue); Serial.println("Amps");
 */
  
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

  delay(1000);

 
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
//------------------------------------------------------------------------------------------------------------
void drawVoltage(void)
{
  oled.set2X();
  oled.setFont(lcdnums14x24);
  oled.setCursor(0,0);
  Serial.print("ADC Value ="); Serial.println(ADCvalue); 
  oled.print(ADCvalue,2);        // viser hele tallet 4½ cifre..overvejer at sløjfe det sidste.
}
//----------------------------------------------------------------------------------------------------------------------
void loop() {
  wdt_reset();     // Watchdog reset
 // ReadTemp();
  ReadDC_from_ADC();
//  ReadAC_from_ADC();
  drawVoltage();   // læs ADCvalue 
// delay(500);
}
//------------------------------------------------------------------------------------------------------------

