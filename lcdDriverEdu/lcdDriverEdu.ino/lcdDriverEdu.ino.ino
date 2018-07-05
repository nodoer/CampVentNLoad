 /*---Shift Register 74HC595---
 * [SR DS]  pin 2
 * [SR ST_CP] pin 3
 * [SR SH_CP]  pin 4


 -----Shift Reg to LCD--------
 * SR Pin 15  - ENABLE        10000000
 * SR Pin 1   - D4            00000010 
 * SR Pin 2   - D5	      00000100
 * SR Pin 3   - D6	      00001000
 * SR Pin 4   - D7	      00010000
 * SR Pin 5   - MOSFET / LED1 00100000
 * SR Pin 6   - LED 2         01000000
 * SR Pin 7   - RS            00000001
 *
 * ----------------------------------------------------------------------------------- */

#include <LiquidCrystal595.h>
#include <Wire.h>
#include "RTClib.h"
#include "BlueDot_BME280.h"

RTC_DS1307 rtc;

// initialize the library with the numbers of the interface pins + the row count
// datapin, latchpin, clockpin, num_lines
LiquidCrystal595 lcd(2,3,4);

int mainDelay = 2000;
int AuxEnPin = 9;   // pin 12 on the 74hc595 latch - nSS 
int SRCLKPin = 6;  // pin 11 on the 74hc595 shift register clock - SCK
int SERPin = 5;    // pin 14 on the 74hc595 data - MOSI

int  KeyEnPin = 7; //Enable pin on 165
int  KeyPLoad = 10; //PLoad on 165

BlueDot_BME280 bme1;                                     //Object for Sensor 1
BlueDot_BME280 bme2;                                     //Object for Sensor 2

int bme1Detected = 0;                                    //Checks if Sensor 1 is available
int bme2Detected = 0;                                    //Checks if Sensor 2 is available

void setup() 
{

pinMode(SERPin, OUTPUT);
pinMode(SRCLKPin, OUTPUT);
pinMode(AuxEnPin, OUTPUT);

//Disable the Key Shift Register
pinMode(KeyEnPin, OUTPUT);
pinMode(KeyPLoad, OUTPUT);
digitalWrite(KeyEnPin, HIGH);
digitalWrite(KeyPLoad, LOW);

//Start the LCD
lcd.begin(16,2);
lcd.setLED2Pin(HIGH);

//Init the SR for Relay and Buzzer
digitalWrite(AuxEnPin, LOW);
shiftOut(SERPin, SRCLKPin, MSBFIRST, B00001111);    
digitalWrite(AuxEnPin, HIGH); 

bme1.parameter.communication = 0;                    //I2C communication for Sensor 1 (bme1)
bme2.parameter.communication = 0;                    //I2C communication for Sensor 2 (bme2)
bme1.parameter.I2CAddress = 0x77;                    //I2C Address for Sensor 1 (bme1)
bme2.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor 2 (bme2)

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
    
  //Now choose on which mode your device will run
  //On doubt, just leave on normal mode, that's the default value

  //0b00:     In sleep mode no measurements are performed, but power consumption is at a minimum
  //0b01:     In forced mode a single measured is performed and the device returns automatically to sleep mode
  //0b11:     In normal mode the sensor measures continually (default value)
  
    bme1.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 1
    bme2.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 2 



  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  
  //Great! Now set up the internal IIR Filter
  //The IIR (Infinite Impulse Response) filter suppresses high frequency fluctuations
  //In short, a high factor value means less noise, but measurements are also less responsive
  //You can play with these values and check the results!
  //In doubt just leave on default

  //0b000:      factor 0 (filter off)
  //0b001:      factor 2
  //0b010:      factor 4
  //0b011:      factor 8
  //0b100:      factor 16 (default value)

    bme1.parameter.IIRfilter = 0b100;                   //IIR Filter for Sensor 1
    bme2.parameter.IIRfilter = 0b100;                   //IIR Filter for Sensor 2

    

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************

    //Next you'll define the oversampling factor for the humidity measurements
  //Again, higher values mean less noise, but slower responses
  //If you don't want to measure humidity, set the oversampling to zero

  //0b000:      factor 0 (Disable humidity measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)

    bme1.parameter.humidOversampling = 0b101;            //Humidity Oversampling for Sensor 1
    bme2.parameter.humidOversampling = 0b101;            //Humidity Oversampling for Sensor 2

    

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  
  //Now define the oversampling factor for the temperature measurements
  //You know now, higher values lead to less noise but slower measurements
  
  //0b000:      factor 0 (Disable temperature measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)

    bme1.parameter.tempOversampling = 0b101;              //Temperature Oversampling for Sensor 1
    bme2.parameter.tempOversampling = 0b101;              //Temperature Oversampling for Sensor 2

    

  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  
  //Finally, define the oversampling factor for the pressure measurements
  //For altitude measurements a higher factor provides more stable values
  //On doubt, just leave it on default
  
  //0b000:      factor 0 (Disable pressure measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)  

    bme1.parameter.pressOversampling = 0b101;             //Pressure Oversampling for Sensor 1
    bme2.parameter.pressOversampling = 0b101;             //Pressure Oversampling for Sensor 2 

     
  
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  
  //For precise altitude measurements please put in the current pressure corrected for the sea level
  //On doubt, just leave the standard pressure as default (1013.25 hPa);
  
    bme1.parameter.pressureSeaLevel = 1013.25;            //default value of 1013.25 hPa (Sensor 1)
    bme2.parameter.pressureSeaLevel = 1013.25;            //default value of 1013.25 hPa (Sensor 2)

  //Also put in the current average temperature outside (yes, really outside!)
  //For slightly less precise altitude measurements, just leave the standard temperature as default (15°C and 59°F);
  
    bme1.parameter.tempOutsideCelsius = 15;               //default value of 15°C
    bme2.parameter.tempOutsideCelsius = 15;               //default value of 15°C
  
    bme1.parameter.tempOutsideFahrenheit = 59;            //default value of 59°F
    bme2.parameter.tempOutsideFahrenheit = 59;            //default value of 59°F



  // Print a message to the LCD.
  lcd.setCursor(0,0);
  lcd.print(" Camp Power LLC ");
  lcd.setCursor(0,1);
  lcd.print("NY   2018   V0.1");
  delay(mainDelay);
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("    Begin     ");
  lcd.setCursor(0,1);
  lcd.print("  Self Test   ");
  delay(mainDelay);
  lcd.clear();


  if (bme1.init() != 0x60)
  {    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BME280 INDOOR  ");
    lcd.setCursor(0,1);
    lcd.print("NOT FOUND!");
    bme1Detected = 0;
    //while(1);
  }

  else
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BME280 INDOOR  ");
    lcd.setCursor(0,1);
    lcd.print("FOUND!");
    delay(mainDelay);
    bme1Detected = 1;
  }

  if (bme2.init() != 0x60)
  {    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BME280 OUTDOOR  ");
    lcd.setCursor(0,1);
    lcd.print("NOT FOUND!");
    bme2Detected = 0;
    //while(1);
  }

  else
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("BME280 OUTDOOR  ");
    lcd.setCursor(0,1);
    lcd.print("FOUND!");
    delay(mainDelay);
    bme2Detected = 1;
  }


   if (! rtc.begin()) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("START UP FAIL!");
    lcd.setCursor(0,0);
    lcd.print("RTC NOT FOUND!");
    //while (1);
   } else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RTC");
    lcd.setCursor(0,1);
    lcd.print("FOUND!");
    delay(mainDelay);
   }

   if (! rtc.isrunning()) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RTC NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    delay(mainDelay);
  }

  lcd.setCursor(0,0);
  lcd.print("   SELF TEST  ");
  lcd.setCursor(0,1);
  lcd.print("    PASS!!!   ");
  delay(mainDelay);
  lcd.clear();
  
  
}

void loop() 
{


  lcd.clear();
  int sensorValue = analogRead(A0);
  float voltageAverage = 0;
  for(int i = 0; i<100; i++){
    voltageAverage = voltageAverage + sensorValue * (21.4 / 1024.0);
  }
  float voltage= voltageAverage/100;
  
  lcd.setCursor(0,0);
  lcd.print("Pb Batt Volts:");
  lcd.setCursor(0,1);
  lcd.print(voltage,1);

  delay(mainDelay);

  lcd.clear();
  DateTime now = rtc.now();
  lcd.setCursor(0,0);
  lcd.print("Date:");
  lcd.setCursor(0,1);
  lcd.print(now.year());
  lcd.setCursor(4,1);
  lcd.print("-");
  lcd.setCursor(5,1);
  lcd.print(now.month());
  lcd.setCursor(7,1);
  lcd.print("-");
  lcd.setCursor(8,1);
  lcd.print(now.day());

  delay(mainDelay);

  
  lcd.clear();
  now = rtc.now();
  lcd.setCursor(0,0);
  lcd.print("Time:");
  lcd.setCursor(0,1);
  lcd.print(now.hour());
  lcd.setCursor(2,1);
  lcd.print(":");
  lcd.setCursor(3,1);
  lcd.print(now.minute());
  lcd.setCursor(5,1);;

  delay(mainDelay);

    digitalWrite(AuxEnPin, LOW);
    shiftOut(SERPin, SRCLKPin, MSBFIRST, B00000110);    
    digitalWrite(AuxEnPin, HIGH); 

  delay(mainDelay);


    digitalWrite(AuxEnPin, LOW);
    shiftOut(SERPin, SRCLKPin, MSBFIRST, B00001001);    
    digitalWrite(AuxEnPin, HIGH); 

      delay(mainDelay);


    digitalWrite(AuxEnPin, LOW);
    shiftOut(SERPin, SRCLKPin, MSBFIRST, B00001111);    
    digitalWrite(AuxEnPin, HIGH); 

  delay(mainDelay);

    digitalWrite(AuxEnPin, LOW);
    shiftOut(SERPin, SRCLKPin, MSBFIRST, B00010000);    
    digitalWrite(AuxEnPin, HIGH); 

      lcd.clear();
  now = rtc.now();
  lcd.setCursor(0,0);
  lcd.print("BEEP");
  
 delay(mainDelay/4);

 digitalWrite(AuxEnPin, LOW);
    shiftOut(SERPin, SRCLKPin, MSBFIRST, B00001111);    
    digitalWrite(AuxEnPin, HIGH); 
  
}

