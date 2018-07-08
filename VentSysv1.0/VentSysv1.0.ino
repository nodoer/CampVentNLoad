/*----------Shift Reg to LCD------------
 * SR Pin 15  - ENABLE       10000000
 * SR Pin 1   - D4           00000010 
 * SR Pin 2   - D5           00000100
 * SR Pin 3   - D6           00001000
 * SR Pin 4   - D7           00010000
 * SR Pin 5   - NC           00100000
 * SR Pin 6   - LED BckLight 01000000
 * SR Pin 7   - RS           00000001
 * ----------------------------------- */

#include <LiquidCrystal595.h>
#include <Wire.h>
#include "RTClib.h"
#include "BlueDot_BME280.h"

/**********************************************
 * System Characteritics
 */
//Determines how wide a clock pulse is for shift registers
#define PULSE_WIDTH_USEC   5

/**********************************************
 * Pin variable Mappings
 */
//Shift Register shared bus
int pinSRClock = 6; //Shift register clock
int pinSRDataOut = 5; //Data in/out pin
int pinSRDataIn = A2;

//Button Shift Register Pins
int  pinBtnSREn = 7; //Enable pin on HC165
int  pinBtnSRPLoad = 10; //PLoad/Shift on HC165 

// LCD Shift Register Pins
int pinLCDData = 2;
int pinLCDEn = 3;
int pinLCDClock = 4;

//Aux Shift Register
int pinAuxSREn = 9;

//

/**********************************************
 * Operational State Structure defs
 */
//Struct to store button state
struct btnState
{
  bool btn1 = false;
  bool btn2 = false;
  bool btn3 = false;
  bool btn4 = false;
  bool btn5 = false;
  bool btn6 = false;
  bool btn7 = false;
  bool btn8 = false;
  bool btn1Last = false;
  bool btn2Last = false;
  bool btn3Last = false;
  bool btn4Last = false;
  bool btn5Last = false;
  bool btn6Last = false;
  bool btn7Last = false;
  bool btn8Last = false;
  bool btnPressed = false;
};

//Struct to store system voltages
struct voltages
{
   float rail3v3 = 0;
   float rail5v = 0;
   float battery = 0;
};

//Struct to store the state of the aux shift register
struct auxState
{
  bool beep = false;

  int rly1MinTimer = 0;
  int rly2MinTimer = 0;
  int rly3MinTimer = 0;
  int rly4MinTimer = 0;

};

//Struct options
struct options{
  //Set to true to turn silence the beep
    boolean beepOff = false;
  //The delay for setup screens in milliseconds
    int mainDelay = 2000;
  //The amount of cycles that will execute before the
  //home screen is returned with no button presses
    int keyPressDelay = 3000;
  //12Volt Calibration value, max range of measurement
    float maxVoltage12 = 20.55;
  //5Volt Calibration value
    float maxVoltage5 = 10.65;
  //The voltage at which the system will stop running loads
    float battCutoffVoltage = 12;
  //The voltage at which the system will start running loads
  //after it has stopped running
    float battTurnOnVoltage = 13.7;
  //The number of seconds the system will beep before
  //shutting down due to a low battery
    byte lowBattShutdownTime = 3;
  //When in scan mode this is the number of cycles each screen 
  //will display for
    int scanModeDelay = 100;
    
  //The max number of hours the vent fan can run  
    byte ventFanMaxRunTime = 6;
  
  
};

//Screen Names
enum screen {
   statusScreen
  ,bypassScreen
  ,dateScreen
  ,timeScreen
  ,relay1Screen
  ,relay2Screen
  ,relay3Screen
  ,relay4Screen
  ,batteryScreen
  ,fiveVoltScreen
  ,threeVoltScreen
  ,battCutOutScreen
  ,battRecoverScreen
  ,outdoorTempC
  ,outdoorTempF
  ,outdoorHumidity
  ,outdoorDewpointC
  ,outdoorDewpointF
  ,outdoorPressure
  ,indoorTempC
  ,indoorTempF
  ,indoorHumidity
  ,indoorDewpointC
  ,indoorDewpointF
  ,indoorPressure
  ,ventRunTimeScreen
  ,dewPointDiffScreen
  
};

byte numOfScreens = 27;

//Operation state struct
struct opState{
  enum screen currentScreen = 0;
   //The screen that will be returned to if a buttton is not
   //pressed within the delay period.
  enum screen timeoutScreen = 0;
  int keyPressTimer = 0;
  float outdoorDewPoint = 0;
  float indoorDewPoint = 0;
  bool scanMode = false;
  int scanTimer = 0;
  bool actionPressed = false; 
  int bypassMinTimer = 0;
  byte currentMinute = 0;
  byte currentDay = 0;
  byte ventRunTimer = 0;
};

typedef struct btnState BtnState;
typedef struct voltages Voltages;
typedef struct auxState AuxState;
typedef struct options Options;
typedef struct opState OpState;


Options opts;
OpState state;
AuxState axState;
BtnState buttons;
Voltages voltState;

//Variable for formating floating numbers
char strFlt[3];
  
DateTime timeState;

/*************************************
 * Init BME280 Sensors
 */
BlueDot_BME280 bme1;                                     //Object for Sensor 1
BlueDot_BME280 bme2;                                     //Object for Sensor 2

int bme1Detected = 0;                                    //Checks if Sensor 1 is available
int bme2Detected = 0;                                    //Checks if Sensor 2 is available

/*************************************
 * Initialize LCD
 */
LiquidCrystal595 lcd(pinLCDData,pinLCDEn,pinLCDClock);

/*************************************
 * Init Real Yime Clock
 */
RTC_DS1307 rtc;

void lcdPrintLines(char line1[16], char line2[16]){
      lcd.setCursor(0,0);
      lcd.print(line1);
      lcd.setCursor(0,1);
      lcd.print(line2);
     
}

void lcdPrintFloatData(char line1[16], float data, char units[11], bool isTemp = false){
      dtostrf(data, 3, 1, strFlt);
      lcd.setCursor(0,0);
      lcd.print(line1);
      lcd.setCursor(0,1);
      lcd.print(strFlt);
      lcd.setCursor(4,1);
      if(isTemp){
        lcd.print((char)223);
      }
      lcd.print(units);
}

void lcdPrintIntData(char line1[16], int data, char units[12]){
      lcd.setCursor(0,0);
      lcd.print(line1);
      lcd.setCursor(0,1);
      lcd.print(data);
      if(data < 10){
        lcd.print("  ");
      } else if (data < 100) {
        lcd.print(" ");
      }
      lcd.setCursor(3,1);
      lcd.print(units);
}

void lcdPrintTime(){
      DateTime timeState = rtc.now();
      lcd.setCursor(0,0);
      lcd.print("Time");
      lcd.setCursor(0,1);
      if(timeState.hour() < 10){
        lcd.print("0");
      }
      lcd.print(timeState.hour());
      lcd.print(":");
      if(timeState.minute() < 10){
        lcd.print("0");
      }
      lcd.print(timeState.minute());
      lcd.print(":");
      if(timeState.second() < 10){
        lcd.print("0");
      }
      lcd.print(timeState.second());
}

void lcdPrintDate(){
      DateTime timeState = rtc.now();
      lcd.setCursor(0,0);
      lcd.print("Date");
      lcd.setCursor(0,1);
      lcd.print(timeState.year());
      lcd.print("-");
      if(timeState.month() < 10){
        lcd.print("0");
      }
      lcd.print(timeState.month());
      lcd.print("-");
      if(timeState.day() < 10){
        lcd.print("0");
      }
      lcd.print(timeState.day());
}

void readButtons(){
 
  //Latch button inputs
  digitalWrite(pinBtnSREn, HIGH);
  digitalWrite(pinBtnSRPLoad, LOW);
  delayMicroseconds(PULSE_WIDTH_USEC);
  digitalWrite(pinBtnSRPLoad, HIGH);
  digitalWrite(pinBtnSREn, LOW);

  bool btnStates[8] = {buttons.btn1,buttons.btn2,buttons.btn3,
                       buttons.btn4,buttons.btn5,buttons.btn6,
                       buttons.btn7, buttons.btn8};

  bool btnLastStates[8] = {buttons.btn1Last,buttons.btn2Last,buttons.btn3Last,
                            buttons.btn4Last,buttons.btn5Last,buttons.btn6Last,
                            buttons.btn7Last, buttons.btn8Last};

  //Read button states from shift register

  buttons.btnPressed = false;
  
  for(int i = 0; i < 8; i++) {
    
        bool bitVal = digitalRead(pinSRDataIn);

        

        btnStates[i] = 0;
        if(bitVal!=btnLastStates[i]){
          btnStates[i] = bitVal;
          btnLastStates[i] = btnStates[i];
          if(bitVal){
            buttons.btnPressed = true;
          }
        }
        
        //Pulse clock to shift next bit out
        digitalWrite(pinSRClock, HIGH);
        delayMicroseconds(PULSE_WIDTH_USEC);
        digitalWrite(pinSRClock, LOW);
    }

    buttons.btn1=btnStates[0];
    buttons.btn2=btnStates[1];
    buttons.btn3=btnStates[2];
    buttons.btn4=btnStates[3];
    buttons.btn5=btnStates[4];
    buttons.btn6=btnStates[5];
    buttons.btn7=btnStates[6];
    buttons.btn8=btnStates[7];

    buttons.btn1Last=btnLastStates[0];
    buttons.btn2Last=btnLastStates[1];
    buttons.btn3Last=btnLastStates[2];
    buttons.btn4Last=btnLastStates[3];
    buttons.btn5Last=btnLastStates[4];
    buttons.btn6Last=btnLastStates[5];
    buttons.btn7Last=btnLastStates[6];
    buttons.btn8Last=btnLastStates[7];

 
  
}

//Updates the state of the relays
void auxUpdate(){

  byte out = B00001111;

  if(axState.rly1MinTimer){
    out = out - B00000001;
  }
 
  if(axState.rly2MinTimer){
    out = out - B00000010;
  }

  if(axState.rly3MinTimer){
    out = out - B00000100;
  }

  if(axState.rly4MinTimer){
    out = out - B00001000;
  }

  if(axState.beep){
    out = out + B00010000;
  }

  digitalWrite(pinAuxSREn, LOW);
  shiftOut(pinSRDataOut, pinSRClock, MSBFIRST, out);    
  digitalWrite(pinAuxSREn, HIGH);  
  
}

float dewpoint(float temp, float humidity){
    double gamma = log(humidity / 100) + ((17.62 * temp) / (243.5 + temp));
    return (243.5 * gamma / (17.62 - gamma));
}


void readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  float railVoltage = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  float railOffset = (railVoltage/1000) - 3.3;

  float v12Factor = (opts.maxVoltage12+railOffset) / 3.3;
  float v5Factor = (opts.maxVoltage5+railOffset) / 3.3;

  delay(2); //wait for analog refs to settle
  float v12Avg = 0;
  for(int i = 0; i<20; i++){
    v12Avg = v12Avg + analogRead(A0);
  }
  float v12Voltage = (v12Avg/20) * ((railVoltage/1000)*v12Factor / 1023.0);


float v5Avg = 0;
  for(int i = 0; i<20; i++){
    v5Avg = v5Avg + analogRead(A1);
  }
  float v5Voltage = (v5Avg/20) * ((railVoltage/1000)*v5Factor / 1023.0);

 voltState.rail3v3 = railVoltage/1000;
 voltState.rail5v = v5Voltage;
 voltState.battery = v12Voltage;
  
}


void batteryCheck(){

  
  if(voltState.battery <= opts.battCutoffVoltage){

    for(int i = opts.lowBattShutdownTime; i > 0; i--){
      lcd.clear();
      lcdPrintIntData("LOW BATTERY!!!!!", i, " seconds left"); 
      axState.beep = true;auxUpdate();
      delay(100);
      axState.beep = false;auxUpdate();
      delay(900);
    }

    //Kill everything the battery is low
      axState.rly1MinTimer = 0;
      axState.rly2MinTimer = 0;
      axState.rly3MinTimer = 0;
      axState.rly4MinTimer = 0;
      axState.beep = false;
      auxUpdate();
    

    do {
      float denom = opts.battTurnOnVoltage-opts.battCutoffVoltage;
      float numer = voltState.battery-opts.battCutoffVoltage;
      float recoPercent = ((numer/denom)*100);

      if(recoPercent<0){
          lcd.clear();
          lcdPrintFloatData("LOW BATTERY", voltState.battery, " volts"); 
      } else {
          lcd.clear();
          lcdPrintIntData("BATT RECOVERING", recoPercent, "% Recovered");
      }
      
      readVcc();
      delay(5000);
      
    } while(voltState.battery <= opts.battTurnOnVoltage);

    lcd.clear();

    
  }
  
}

void setup() {

  //Set pin Modes
  pinMode(pinSRClock,OUTPUT);
  pinMode(pinSRDataOut,OUTPUT);
  pinMode(pinSRDataIn,INPUT);
  pinMode(pinBtnSREn,OUTPUT);
  pinMode(pinBtnSRPLoad, OUTPUT);
  pinMode(pinAuxSREn, OUTPUT);


  //StartUp LCD
  lcd.begin(16,2);
  lcd.setLED1Pin(HIGH);
  lcd.setLED2Pin(HIGH);

  //Setup BME280 Sensors
  bme1.parameter.communication = 0;                    //I2C communication for Sensor 1 (bme1)
  bme2.parameter.communication = 0;                    //I2C communication for Sensor 2 (bme2)
  bme1.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor 1 (bme1)
  bme2.parameter.I2CAddress = 0x77;                    //I2C Address for Sensor 2 (bme2)
  bme1.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 1 0b11 Normal 0b01 Force (Could save power)
  bme2.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 2 
  bme1.parameter.IIRfilter = 0b100;                    //IIR Filter for Sensor 1
  bme2.parameter.IIRfilter = 0b100;                    //IIR Filter for Sensor 2
  bme1.parameter.humidOversampling = 0b101;            //Humidity Oversampling for Sensor 1
  bme2.parameter.humidOversampling = 0b101;            //Humidity Oversampling for Sensor 2
  bme1.parameter.tempOversampling = 0b101;             //Temperature Oversampling for Sensor 1
  bme2.parameter.tempOversampling = 0b101;             //Temperature Oversampling for Sensor 2
  bme1.parameter.pressOversampling = 0b101;            //Pressure Oversampling for Sensor 1
  bme2.parameter.pressOversampling = 0b101;            //Pressure Oversampling for Sensor 2 
  bme1.parameter.pressureSeaLevel = 1013.25;            //default value of 1013.25 hPa (Sensor 1)
  bme2.parameter.pressureSeaLevel = 1013.25;            //default value of 1013.25 hPa (Sensor 2)
  bme1.parameter.tempOutsideCelsius = 15;               //default value of 15°C
  bme2.parameter.tempOutsideCelsius = 15;               //default value of 15°C
  bme1.parameter.tempOutsideFahrenheit = 59;            //default value of 59°F
  bme2.parameter.tempOutsideFahrenheit = 59;            //default value of 59°F
  

  //Init the SR for Relay and Buzzer
  digitalWrite(pinAuxSREn, LOW);
  shiftOut(pinSRDataOut , pinSRClock, MSBFIRST, B00001111);    
  digitalWrite(pinAuxSREn, HIGH); 

  

  // Print a message to the LCD.
  lcd.clear();
  lcdPrintLines("Shiroda Power Co", "2018        V1.0");
  delay(opts.mainDelay);

  //////////////////////////////////////////
  // Start Self Test
  //////////////////////////////////////////
  lcd.clear();
  lcdPrintLines("Begin", "Self Testing");
  axState.beep = true;auxUpdate();
  delay(200);
  axState.beep = false;auxUpdate();
  delay(200);
  axState.beep = true;auxUpdate();
  delay(500);
  axState.beep = false;auxUpdate();
  delay(opts.mainDelay);

  //Start and Check for RTC
  if (! rtc.begin()) {
    lcd.clear();
    lcdPrintLines("RTC Test Failed!", " RTC NOT FOUND! ");
    axState.beep = true;auxUpdate();
    delay(500);
    axState.beep = false;auxUpdate();
    while (1);
   } else {
    lcd.clear();
    lcdPrintLines("RTC Test Passed!", "   RTC FOUND!   ");
    delay(opts.mainDelay);
   }

 //Check to see if the RTC is running if not set the time
 if (! rtc.isrunning()) {
    axState.beep = true;auxUpdate();
    delay(500);
    axState.beep = false;auxUpdate();
    lcd.clear();
    lcdPrintLines("RTC !!WARNING!! ", " Time is wrong! ");
    delay(opts.mainDelay/2);
    lcd.clear();
    lcdPrintLines(" Time is wrong! ", " Check RTC Batt ");
    delay(opts.mainDelay*2);
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

 //Start and Check for BME280 1 (Outdoor)
 if (bme1.init() != 0x60){
    lcd.clear();
    lcdPrintLines("BME  280 OUTDOOR", "   NOT FOUND!   ");
    axState.beep = true;auxUpdate();
    delay(500);
    axState.beep = false;auxUpdate();
    while (1);
 } else {
    lcd.clear();
    lcdPrintLines("BME  280 OUTDOOR", "     FOUND!     ");
    delay(opts.mainDelay);
    bme1Detected = 1;
 }

  //Start and Check for BME280 2 (Indoor)
 if (bme2.init() != 0x60){
    lcd.clear();
    lcdPrintLines("BME  280  INDOOR", "   NOT FOUND!   ");
    axState.beep = true;auxUpdate();
    delay(500);
    axState.beep = false;auxUpdate();
    while (1);
 } else {
    lcd.clear();
    lcdPrintLines("BME  280  INDOOR", "     FOUND!     ");
    delay(opts.mainDelay);
    bme2Detected = 1;
 }


 lcd.clear();
 lcdPrintLines("Self Testing", "Complete");
 delay(opts.mainDelay);
 lcd.clear();
  
}

void updateTimers(){

  if(state.bypassMinTimer > 0){
    state.bypassMinTimer--;
  }

  if(axState.rly1MinTimer > 0){
    axState.rly1MinTimer--;
  }
  
  if(axState.rly2MinTimer > 0){
    axState.rly2MinTimer--;
  }

  if(axState.rly3MinTimer > 0){
    axState.rly3MinTimer--;
    if (axState.rly3MinTimer < 2){
      lightsOutBeep();
      lightsOutBeep();
      lightsOutBeep();
      state.currentScreen = relay3Screen;
    } else if (axState.rly3MinTimer < 5){
      lightsOutBeep();
      lightsOutBeep();
      state.currentScreen = relay3Screen;
    }else if (axState.rly3MinTimer < 10){
      lightsOutBeep();
      state.currentScreen = relay3Screen;
    }
  }

  if(axState.rly4MinTimer > 0){
    axState.rly4MinTimer--;
    if (axState.rly4MinTimer < 2){
      lightsOutBeep();
      lightsOutBeep();
      lightsOutBeep();
      state.currentScreen = relay4Screen;
    } else if (axState.rly4MinTimer < 5){
      lightsOutBeep();
      lightsOutBeep();
      state.currentScreen = relay4Screen;
    }else if (axState.rly4MinTimer < 10){
      lightsOutBeep();
      state.currentScreen = relay4Screen;
    }
  }
  
}

void lightsOutBeep(){
  beep(100);
  beep(100);
  beep(100);
  beep(200);
}

void beep(int beepLen){
    axState.beep = true;auxUpdate();
    delay(beepLen/2);
    axState.beep = false;auxUpdate();
    delay(beepLen/2);
}


void lcdNoAction(){
  if(state.actionPressed){
    lcdPrintLines("No action on", "this screen.");
    delay(500);
    lcd.clear();
  }
}

void loop() {

  DateTime timeState = rtc.now();

  readButtons();
  auxUpdate();
  readVcc();
  batteryCheck();
  

  state.outdoorDewPoint = dewpoint(bme1.readTempC(), bme1.readHumidity());
  state.indoorDewPoint = dewpoint(bme2.readTempC(), bme2.readHumidity());

  //if the minute has changed update all of the minute counters
  if(timeState.second() != state.currentMinute){
    updateTimers();
    state.currentMinute = timeState.second();
  }

  //if the day has changed reset daily counters
  if(timeState.day() != state.currentDay){
    state.ventRunTimer = 0;
    state.currentDay = timeState.day();
    
  }
  

  //Check to see if the fans should be on because of dew point
  if (!state.bypassMinTimer && opts.ventFanMaxRunTime != state.ventRunTimer){

    //Added one to make the transition more stable
    if((state.outdoorDewPoint+1) < state.indoorDewPoint 
        && !axState.rly1MinTimer
        && !axState.rly2MinTimer){

          state.ventRunTimer++;
          axState.rly1MinTimer = 60;
          axState.rly2MinTimer = 60;
          
    }
    
  }

  //if scan mode is on check the timer and switch screens
  if(state.scanMode){
    state.keyPressTimer = 0;
    if(state.scanTimer == opts.scanModeDelay){
      state.currentScreen = state.currentScreen+1;
      state.scanTimer = 0;
      lcd.clear();
    }

    state.scanTimer++;
  }
  
  //Make sure the screens do not go past the max
  if(state.currentScreen > numOfScreens-1){
    state.currentScreen = 0;
  } else if (state.currentScreen < 0){
    state.currentScreen = numOfScreens-1;
  }
  

  //Resets the display if key presses stop
  if(opts.keyPressDelay == state.keyPressTimer){
    state.currentScreen = state.timeoutScreen;
    lcd.setLED2Pin(LOW);
  } else {
    state.keyPressTimer++;
  }

  switch (state.currentScreen){

    case statusScreen:
    lcdPrintLines("System ON       ", (state.bypassMinTimer)?"Bypass ON       ":"                ");         
    break;

    case bypassScreen:
    if(state.bypassMinTimer){
      lcdPrintFloatData("Bypass Mode", (float)state.bypassMinTimer/60, " hours left.");
    } else {
      lcdPrintLines("Bypass mode", "is off.         ");
    }

    if(state.actionPressed){
      state.bypassMinTimer = state.bypassMinTimer + 1440;
      if (state.bypassMinTimer > 5760 ){
        state.bypassMinTimer = 0;
      }
    } 
             
    break;

    case dateScreen:
    lcdPrintDate();
    lcdNoAction();
    break;

    case timeScreen:
    lcdPrintTime();
    lcdNoAction();
    break;

    case relay1Screen:
    if(axState.rly1MinTimer){
      lcdPrintIntData("Large Vent Fan", axState.rly1MinTimer , " mins left");
    } else {
      lcdPrintLines("Large Vent Fan", "is off.        ");
    }
    
    if(state.actionPressed){

      axState.rly1MinTimer = axState.rly1MinTimer + 10;
      if(axState.rly1MinTimer > 130){
        axState.rly1MinTimer = 120;
      }
    }
    break;

    case relay2Screen:
    if(axState.rly2MinTimer){
      lcdPrintIntData("Circulate Fans", axState.rly2MinTimer , " mins left");
    } else {
      lcdPrintLines("Circulate Fans", "are off.        ");
    }
    
    if(state.actionPressed){

      axState.rly2MinTimer = axState.rly2MinTimer + 10;
      if(axState.rly2MinTimer > 130){
        axState.rly2MinTimer = 120;
      }
    }
    break;

    case relay3Screen:
    if(axState.rly3MinTimer){
      lcdPrintFloatData("Kitchen Lights", (float)axState.rly3MinTimer/60 , " hours left");
    } else {
      lcdPrintLines("Kitchen Lights", "are off.         ");
    }

    if(state.actionPressed){
      axState.rly3MinTimer = axState.rly3MinTimer + 720;
      if (axState.rly3MinTimer > 5760 ){
        axState.rly3MinTimer = 0;
      }
    } 

    break;

    case relay4Screen:
    if(axState.rly4MinTimer){
      lcdPrintFloatData("Bedroom Lights", (float)axState.rly4MinTimer/60 , " hours left");
    } else {
      lcdPrintLines("Bedroom Lights", "are off.         ");
    }

    if(state.actionPressed){
      axState.rly4MinTimer = axState.rly4MinTimer + 720;
      if (axState.rly4MinTimer > 5760 ){
        axState.rly4MinTimer = 0;
      }
    } 
    
    break;

    case batteryScreen:
    lcdPrintFloatData("Pb Batt Voltage", voltState.battery, " volts");
    break;

    case fiveVoltScreen:
    lcdPrintFloatData("5 Volt Rail", voltState.rail5v, " volts");
    break;

    case threeVoltScreen:
    lcdPrintFloatData("3v3 Volt Rail", voltState.rail3v3, " volts");
    break;

    case battCutOutScreen:
    lcdPrintFloatData("Batt cuts off at", opts.battCutoffVoltage, " volts");
    break;

    case battRecoverScreen:
    lcdPrintFloatData("Batt recovers at", opts.battTurnOnVoltage, " volts");
    break;

    case outdoorTempC:
    lcdPrintFloatData("Outdoor Temp", bme1.readTempC(), "C", true);
    break;

    case outdoorTempF:
    lcdPrintFloatData("Outdoor Temp", bme1.readTempF(), "F", true);
    break;

    case outdoorHumidity:
    lcdPrintIntData("Outdoor Humidity", bme1.readHumidity(), "%"); 
    break;

    case outdoorDewpointC:
    lcdPrintFloatData("Outdoor Dewpoint", state.outdoorDewPoint, "C", true);
    break;

    case outdoorDewpointF:
    lcdPrintFloatData("Outdoor Dewpoint", (state.outdoorDewPoint * (9/5) + 32), "F", true);
    break;

    case outdoorPressure:
    lcdPrintFloatData("Outdoor Pressure", bme1.readPressure()," hPa");
    break;

    case indoorTempC:
    lcdPrintFloatData("Indoor Temp", bme2.readTempC(), "C", true);
    break;

    case indoorTempF:
    lcdPrintFloatData("Indoor Temp", bme2.readTempF(), "F", true);
    break;

    case indoorHumidity:
    lcdPrintIntData("Indoor Humidity", bme2.readHumidity(), "%"); 
    break;

    case indoorDewpointC:
    lcdPrintFloatData("Indoor Dewpoint", state.indoorDewPoint, "C", true);
    break;

    case indoorDewpointF:
    lcdPrintFloatData("Indoor Dewpoint", (state.indoorDewPoint * (9/5) + 32), "F", true);
    break;

    case indoorPressure:
    lcdPrintFloatData("Indoor Pressure", bme2.readPressure()," hPa");
    break;

    case ventRunTimeScreen:
    lcdPrintIntData("Fans have run   ", state.ventRunTimer , " times today");
    break;

    case dewPointDiffScreen:
    lcdPrintFloatData("Dewpoint diff is", state.outdoorDewPoint - state.indoorDewPoint , "C", true);
    break;
    
  }

  state.actionPressed = false;
  if(buttons.btnPressed){
    lcd.clear();

    //Reset key press timer
    state.keyPressTimer = 0;

    //LCD Back Light ON
    lcd.setLED2Pin(HIGH);

    if(buttons.btn3){
      state.currentScreen = state.currentScreen+1;
    }
    if(buttons.btn2){
      state.currentScreen = state.currentScreen-1;
    }

    if(buttons.btn8){
      state.currentScreen = relay1Screen;
      if(axState.rly1MinTimer){
        axState.rly1MinTimer = 0;
      } else {
        axState.rly1MinTimer = 60;
      }
    }

    if(buttons.btn7){
      state.currentScreen = relay2Screen;
      if(axState.rly2MinTimer){
        axState.rly2MinTimer = 0;
      } else {
        axState.rly2MinTimer = 60;
      }
    }

    if(buttons.btn6){
      state.currentScreen = relay3Screen;
      if(axState.rly3MinTimer){
        axState.rly3MinTimer = 0;
      } else {
        axState.rly3MinTimer = 720;
      }
    }

    if(buttons.btn5){
      state.currentScreen = relay4Screen;
      if(axState.rly4MinTimer){
        axState.rly4MinTimer = 0;
      } else {
        axState.rly4MinTimer = 720;
      }
    }

    if(buttons.btn1){
      if(state.scanMode){
        state.scanMode = false;
        lcd.clear();
        lcdPrintLines("Scan Mode Off", "");
        delay(500);
        lcd.clear();
      } else {
        state.scanMode = true;
        lcd.clear();
        lcdPrintLines("Scan Mode On", "");
        delay(500);
        lcd.clear();
      }
      
    }

    
    if(buttons.btn4){
      state.actionPressed = true;
    }
  
  }
  
  
  
}







