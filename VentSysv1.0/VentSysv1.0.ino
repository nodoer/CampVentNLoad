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
  bool rly1 = false;
  bool rly2 = false;
  bool rly3 = false;
  bool rly4 = false;
  bool beep = false;
  bool aux1 = false;
  bool aux2 = false;
};

//Struct options
struct options{
  //Set to true to turn silence the beep
  boolean beepOff = false;
  int mainDelay = 2000;
  //The amount of cycles that will execute before the
  //home screen is returned with no button presses
  int keyPressDelay = 400;
  //12Volt Calibration value, max range of measurement
  float maxVoltage12 = 20.55;
  float maxVoltage5 = 10.65;
  int voltCheckInterval = 50;
  float battCutoffVoltage = 12;
  float battTurnOnVoltage = 13.7;
  int lowBattShutdownTime = 5;
  
};

//Screen Names
enum screen {
   statusScreen
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
};

//Operation state struct
struct opState{
  enum screen currentScreen = 0;
   //The screen that will be returned to if a buttton is not
   //pressed within the delay period.
  enum screen timeoutScreen = 0;
  int keyPressTimer = 0;
  int voltCheckTimer = 0;
  char lcdLine1[16];
  char lcdLine2[16];
  char lcdOldLine1[16];
  char lcdOldLine2[16];
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

void lcdUpdate(){

    if(state.lcdLine1 != state.lcdOldLine1){
      lcd.setCursor(0,0);
      lcd.print(state.lcdLine1);
    }

    if(state.lcdLine1 != state.lcdOldLine1){
      lcd.setCursor(0,1);
      lcd.print(state.lcdLine2);
    }
  
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
  
  for(int i = 0; i < 8; i++) {
    
        bool bitVal = digitalRead(pinSRDataIn);

        if (bitVal){
          buttons.btnPressed = true;
        }

        btnStates[i] = 0;
        if(bitVal!=btnLastStates[i]){
          btnStates[i] = bitVal;
          btnLastStates[i] = btnStates[i];
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

  if(axState.rly1){
    out = out - B00000001;
  }
 
  if(axState.rly2){
    out = out - B00000010;
  }

  if(axState.rly3){
    out = out - B00000100;
  }

  if(axState.rly4){
    out = out - B00001000;
  }

  if(axState.beep){
    out = out + B00010000;
  }

  digitalWrite(pinAuxSREn, LOW);
  shiftOut(pinSRDataOut, pinSRClock, MSBFIRST, out);    
  digitalWrite(pinAuxSREn, HIGH);  
  
}


void setup() {

  //Set pin Modes
  pinMode(pinSRClock,OUTPUT);
  pinMode(pinSRDataOut,OUTPUT);
  pinMode(pinSRDataIn,INPUT);
  pinMode(pinBtnSREn,OUTPUT);
  pinMode(pinBtnSRPLoad, OUTPUT);
  pinMode(pinAuxSREn, OUTPUT);

  //Do a volt check as soon as the loop starts
  state.voltCheckTimer = opts.voltCheckInterval;

  //StartUp LCD
  lcd.begin(16,2);
  lcd.setLED1Pin(HIGH);
  lcd.setLED2Pin(HIGH);

  //Setup BME280 Sensors
  bme1.parameter.communication = 0;                    //I2C communication for Sensor 1 (bme1)
  bme2.parameter.communication = 0;                    //I2C communication for Sensor 2 (bme2)
  bme1.parameter.I2CAddress = 0x77;                    //I2C Address for Sensor 1 (bme1)
  bme2.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor 2 (bme2)
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
  sprintf(state.lcdLine1,"Shiroda Power Co");
  sprintf(state.lcdLine2,"2018        V1.0");
  lcdUpdate();
  delay(opts.mainDelay);

  //////////////////////////////////////////
  // Start Self Test
  //////////////////////////////////////////
  sprintf(state.lcdLine1,"Begin           ");
  sprintf(state.lcdLine2,"Self Testing    ");
  lcdUpdate();
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
    sprintf(state.lcdLine1,"RTC Test Failed!");
    sprintf(state.lcdLine2," RTC NOT FOUND! ");
    lcdUpdate();
    axState.beep = true;auxUpdate();
    delay(500);
    axState.beep = false;auxUpdate();
    while (1);
   } else {
    sprintf(state.lcdLine1,"RTC Test Passed!");
    sprintf(state.lcdLine2,"   RTC FOUND!   ");
    lcdUpdate();
    delay(opts.mainDelay);
   }

 //Check to see if the RTC is running if not set the time
 if (! rtc.isrunning()) {
    axState.beep = true;auxUpdate();
    delay(500);
    axState.beep = false;auxUpdate();
    sprintf(state.lcdLine1,"RTC !!WARNING!! ");
    sprintf(state.lcdLine2," Time is wrong! ");
    lcdUpdate();
    delay(opts.mainDelay/2);
    sprintf(state.lcdLine1," Time is wrong! ");
    sprintf(state.lcdLine2," Check RTC Batt ");
    lcdUpdate();
    delay(opts.mainDelay*2);
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

 //Start and Check for BME280 1 (Outdoor)
 if (bme1.init() != 0x60){
    sprintf(state.lcdLine1,"BME  280 OUTDOOR");
    sprintf(state.lcdLine2,"   NOT FOUND!   ");
    lcdUpdate();
    axState.beep = true;auxUpdate();
    delay(500);
    axState.beep = false;auxUpdate();
    //while (1);
 } else {
    sprintf(state.lcdLine1,"BME  280 OUTDOOR");
    sprintf(state.lcdLine2,"     FOUND!     ");
    lcdUpdate();
    delay(opts.mainDelay);
    bme2Detected = 1;
 }


   
 sprintf(state.lcdLine1,"Self Testing    ");
 sprintf(state.lcdLine2,"PASS!           ");
 lcdUpdate();
 delay(opts.mainDelay);
  

}




void loop() {

  //Variable for formating floating numbers
  char strFlt[3];
 
  lcdUpdate();

  readButtons();

  auxUpdate();

  DateTime timeState = rtc.now();

  state.voltCheckTimer++;
  if(state.voltCheckTimer >= opts.voltCheckInterval){
    state.voltCheckTimer = 0;
    readVcc();
    batteryCheck();
  }
  


  if(buttons.btnPressed){

    buttons.btnPressed = false;

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
      if(axState.rly1){
        axState.rly1 = false;
      } else {
        axState.rly1 = true;
        
      }
    }
  
  }


  if(opts.keyPressDelay == state.keyPressTimer){
    state.currentScreen = state.timeoutScreen;
    lcd.setLED2Pin(LOW);
  } else {
    state.keyPressTimer++;
  }

  switch (state.currentScreen){

    case statusScreen:
      sprintf(state.lcdLine1,"Status         ");
      sprintf(state.lcdLine2,"Normal %d      ", state.keyPressTimer);

    break;

    case dateScreen:
      sprintf(state.lcdLine1,"Date            ");
      sprintf(state.lcdLine2,"%4d-%02d-%02d      ", timeState.year()
                , timeState.month(), timeState.day());
    break;

    case timeScreen:
      sprintf(state.lcdLine1,"Time            ");
      sprintf(state.lcdLine2,"%02d:%02d:%02d      ", timeState.hour()
                , timeState.minute(), timeState.second());
    break;

    case relay1Screen:
      sprintf(state.lcdLine1,"Large Vent Fan  ");
      sprintf(state.lcdLine2,"Status: %s      ", (axState.rly1)?"ON":"OFF");
    break;

    case relay2Screen:
      sprintf(state.lcdLine1,"Circulate Fans  ");
      sprintf(state.lcdLine2,"Status: %s      ", (axState.rly2)?"ON":"OFF");
    break;

    case relay3Screen:
      sprintf(state.lcdLine1,"Kitchen Lights  ");
      sprintf(state.lcdLine2,"Status: %s      ", (axState.rly3)?"ON":"OFF");
    break;

    case relay4Screen:
      sprintf(state.lcdLine1,"Bedroom Lights  ");
      sprintf(state.lcdLine2,"Status: %s      ", (axState.rly3)?"ON":"OFF");
    break;

    case batteryScreen:
      dtostrf(voltState.battery, 3, 1, strFlt);
      sprintf(state.lcdLine1,"Pb Batt Voltage ");
      sprintf(state.lcdLine2,"%s volts        ", strFlt);
      
    break;

    case fiveVoltScreen:
      dtostrf(voltState.rail5v, 3, 1, strFlt);
      sprintf(state.lcdLine1,"5 Volt Rail     ");
      sprintf(state.lcdLine2,"%s volts        ", strFlt);
    break;

    case threeVoltScreen:
      dtostrf(voltState.rail3v3, 3, 1, strFlt);
      sprintf(state.lcdLine1,"3v3 Volt Rail   ");
      sprintf(state.lcdLine2,"%s volts        ", strFlt);
    break;

    case battCutOutScreen:
      sprintf(state.lcdLine1,"Batt cuts off at");
      sprintf(state.lcdLine2,"%d volts.       ", opts.battCutoffVoltage);
    break;

    case battRecoverScreen:
      sprintf(state.lcdLine1,"Batt recovers at");
      sprintf(state.lcdLine2,"%d volts        ", opts.battTurnOnVoltage);
    break;
    
  }
  
  
  
}


void batteryCheck(){

  
  if(voltState.battery <= opts.battCutoffVoltage || true){

    lcdUpdate();

    for(int i = opts.lowBattShutdownTime; i > 0; i--){
      axState.beep = true;auxUpdate();
      delay(100);
      axState.beep = false;auxUpdate();
      delay(900);
      sprintf(state.lcdLine1,"LOW BATTERY!!!!!");
      sprintf(state.lcdLine2,"%d seconds left ", i);
      lcdUpdate();
    }

    //Kill everything the battery is low
      axState.rly1 = false;
      axState.rly2 = false;
      axState.rly3 = false;
      axState.rly4 = false;
      axState.beep = false;
      axState.aux1 = false;
      axState.aux2 = false;
      auxUpdate;
    

    do {
      char percent[3];
      float denom = opts.battTurnOnVoltage-opts.battCutoffVoltage;
      float numer = opts.battTurnOnVoltage-voltState.battery;
      dtostrf(((numer/denom)*100), 3, 0, percent);
      sprintf(state.lcdLine1,"LOW BATT RECOVER");
      sprintf(state.lcdLine2,"%s%% Recovered  ", percent);
      lcdUpdate();
      readVcc();
      delay(5000);
      
    } while(voltState.battery <= opts.battTurnOnVoltage);

    
  }
  
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



