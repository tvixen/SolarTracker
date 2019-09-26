
//How to measure AC current with the DL-CT08CL5-20A/10mA 2000/1 0~120A Micro Current Transformer HH.

//Make a little circuit of 3 resistors and a cap(10uf) + the coil.
//Size of R3 is  primary current divided with the turn ratio.  Mine is 20A 2000T:  20/2000=0,01 or 0,01mA.
//Reference from R1 and R2 (both 10K) gives 2.5volt, so 2.5/0,01= 250ohm. So the burden resistor should be 250ohm, but I should measure it!!!!  
// drawing: https://www.the-diy-life.com/simple-arduino-home-energy-meter/
// look also here: https://learn.openenergymonitor.org/electricity-monitoring/voltage-sensing/measuring-voltage-with-an-acac-power-adapter


int currentPin = 1;              //Assign CT input to pin 1
double kilos = 0;
int peakPower = 0;
#define LysnetAC 230

void setup() 
{ 
  Serial.begin(115200);            //Start serial communication
  Serial.println("Running");
}

void loop() 
{ 
  int current = 0;
  int maxCurrent = 0;
  int minCurrent = 1000;
  
  unsigned long startMillis;
  unsigned long endMillis;

  for (int i=0 ; i<=200 ; i++)  //Monitors and logs the current input for 200 cycles to determine max and min current
  {
    current = analogRead(currentPin);    //Reads current input and records maximum and minimum current
    if(current >= maxCurrent)
      maxCurrent = current;
    else if(current <= minCurrent)
      minCurrent = current;
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

  delay (2000);
  Serial.print(RMSCurrent);
  Serial.println("A");
  Serial.print(RMSPower);
  Serial.println("W");
  Serial.print(kilos);
  Serial.println("kWh");
  Serial.print(peakPower);
  Serial.println("W");
}


