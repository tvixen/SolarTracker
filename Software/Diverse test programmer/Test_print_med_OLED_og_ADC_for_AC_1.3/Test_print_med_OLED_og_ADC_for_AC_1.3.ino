/*
 * 
 * 24/11-2018
 * Denne kode virker er til test printet hvor der er monteret en blå diode samt OLE og ADS1115.
 * Til små forsøgsopstillinger.
 * Her en strøm opstilling med AC ZDCT004GL 2000/1 turn med 330 ohm over. samt 1 stk 12ohm i serie i hver ledning.
 * Har monteret en 330ohm modstand over spolen som load. hvilker giver:
 * 100w 72.1mv, 200w 144,2mv
 * 
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

  Serial.println(" Init OLED");
  initOled();      // Oled display startup
  Serial.println(" Init ADC");
  initADC();      // kald ads1115.begin();
  Serial.println(" Setup done version Test_print_med_OLED_og_ADC_1.3");
  Serial.println(" Ready to read AC on A0 + A1   w. 200ohm load ");
}
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
  
   ads1115CurrentModule.setGain(GAIN_TWO);  //Gain to multiply with
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
  Serial.println();
  Serial.println("ReadAC_from_ADC()  ");

  //const float multiplier = 0.0625F;   //TWO
  const float multiplier = 0.03125;   //GAIN_FOUR
 //const float multiplier = 0.015625; //Eight 
  #define LysnetAC 230

  long  StartTime = millis();
  
  double current = 0;
  double maxCurrent = 0;
  double minCurrent = 1000;
  
  unsigned long startMillis;
  unsigned long endMillis;
  while (millis() - StartTime < 1000)  //measure 1.sec
 {
    current = ads1115CurrentModule.readADC_Differential_2_3();
    if(current >= maxCurrent)
      maxCurrent = current;
    else if(current <= minCurrent)
      minCurrent = current;
  }
  if (maxCurrent <= 517)
  {
    maxCurrent = 516;
  }


double RMSCurrentU = ((maxCurrent - 516)*0.707);  
  Serial.print("RMSCurrentU = "); Serial.print(RMSCurrentU ); Serial.println(" A");
  
  double RMSCurrent = ((maxCurrent - 516)*0.707)/2679.4355;    //Calculates RMS current based on maximum value. Adjust 11.8337 to be more precise. do it with a known bulb (40w)
  int RMSPower = LysnetAC*RMSCurrent;    //Calculates RMS Power Assuming Voltage 230VAC, change to 220VAC accordingly


  Serial.print("maxCurrent: "); Serial.println(maxCurrent); 
  Serial.print("minCurrent: "); Serial.println(minCurrent); 
  Serial.print("RMSCurrent: "); Serial.println(RMSCurrent); 
  Serial.print("Watt = "); Serial.print(RMSCurrent * 230); Serial.println(" W");
 
Serial.println("------------------------------------------------------------");
}
//------------------------------------------------------------------------------------------------------------

void ReadACCurrentADC() 
{
  
  
  
  #define LysnetAC 230
  
  Serial.println();
  Serial.println("ReadACCurrentADC() with multiplier ");
  
  float currentRMS = GetCurrent(); //call function GetCurrent
  if (currentRMS<=0.002) currentRMS=0.00F;

  float power = LysnetAC * currentRMS;
  

  Serial.print("CurrentRMS = "); Serial.print(currentRMS,3); Serial.println(" A");

  
  Serial.print("Watt = "); Serial.print(currentRMS * LysnetAC); Serial.println(" W");
  ADCvalue=currentRMS;
  Serial.println("------------------------------------------------------------");

}

// **********subcall to ReadACCurrentADC***************************
float GetCurrent()
{

 const float multiplier = 0.0625F;   //TWO
 // const float multiplier = 0.03125;   //GAIN_FOUR
 //const float multiplier = 0.015625; //Eight 
 
 const float FACTOR     = 2.50F;  //30A 1V  32=0.11A(ok) 32=0.33(ej ok)
 float Voltage          = 0.0F ;
 float Current          = 0.0F ;
 float Sum              = 0.0F;
 long  StartTime = millis();
 int   Counter = 0;

// 100w 230v = 447ma Fluke / 0.001V Voltcraft over 47ohm / 

  int maxCurrent = 0;
  int minCurrent = 100;

 while (millis() - StartTime < 1000)  //measure 1.sec  367 omgange
  {
    Counter = Counter +1;
    Voltage = ads1115CurrentModule.readADC_Differential_2_3() * multiplier;  //1285 ved 100www
    if(Voltage >= maxCurrent)
      maxCurrent = Voltage;
    else if(Voltage <= minCurrent)
      minCurrent = Voltage;
    Current = Voltage * FACTOR;
    Current /= 1000.0;
    Sum+= sq(Current);
    ads1115CurrentModule.waitForConversion();
  }
 Serial.print("maxCurrent  "); Serial.println(maxCurrent);
 Serial.print("minCurrent  "); Serial.println(minCurrent);
 Serial.print("Counter     "); Serial.println(Counter);
 Serial.print("Voltage     "); Serial.println(Voltage/multiplier);
 Serial.println("------------");

  

 Current = sqrt(Sum / Counter);
 return(Current);
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
 // Serial.print("ADC Value ="); Serial.println(ADCvalue); 
  oled.print(ADCvalue,2);        // viser hele tallet 4½ cifre..overvejer at sløjfe det sidste.
  
  oled.setFont(Verdana12);
  oled.set1X();
  oled.setCursor(2,6);
  //oled.setScroll(true);
  for (int i = 0; i <= 50; i=i+10) 
  {
    oled.setCursor(i,6);
    oled.print("     Made By Tim Milgart");  
    
    delay(100);
  }
  
}
//------------------------------------------------------------------------------------------------------------
void loop() {
  wdt_reset();     // Watchdog reset
 // ReadTemp();
//  ReadDC_from_ADC();
//  ReadAC_from_ADC();
 
ReadACCurrentADC();
 
  
  drawVoltage();   // læs ADCvalue 
// delay(500);
}
 
