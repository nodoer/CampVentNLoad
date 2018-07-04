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
int pinSRData = 5; //Data in/out pin
 
//Button Shift Register Pins
int  pinBtnSREn = 7; //Enable pin on HC165
int  pinBtnSRPLoad = 10; //PLoad/Shift on HC165 

// LCD Shift Register Pins
int pinLCDData = 2;
int pinLCDEn = 3;
int pinLCDClock = 4;

//Aux Shift Register
int pinAuxSREn = 9;

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
  int keyPressDelay = 10000;
};

//Screen Names
enum screen {
   statusScreen
  ,dateScreen
  ,timeScreen
};

//Operation state struct
struct opState{
  enum screen currentScreen = 0;
   //The screen that will be returned to if a buttton is not
   //pressed within the delay period.
  enum screen timeoutScreen = 0;
  int keyPressTimer = 0;
  char lcdLine1[16];
  char lcdLine2[16];
  char lcdOldLine1[16];
  char lcdOldLine2[16];
};

//Ambient weather struct
struct weather{
  float inDewPoint = 0;
  float inTemp = 0;
  float inHumidity = 0;
  float inPressure = 0;

  float outDewPoint = 0;
  float outTemp = 0;
  float outHumidity = 0;
  float outPressure = 0;
};

struct datetime{
  int curHour;
  int curMin;
  int curSecond;
  int curYear;
  int curMonth;
  int curDay;
};

typedef struct btnState BtnState;
typedef struct voltages Voltages;
typedef struct auxState AuxState;
typedef struct options Options;
typedef struct opState OpState;
typedef struct weather Weather;
typedef struct datetime Datetime;


Options opts;
OpState state;


/*************************************
 * Initialize LCD
 */
LiquidCrystal595 lcd(pinLCDData,pinLCDEn,pinLCDClock);

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

btnState readButtons(){

  btnState pressedBtns;
  
  pinMode(pinSRData,INPUT);

  //Latch button inputs
  digitalWrite(pinBtnSREn, HIGH);
  digitalWrite(pinBtnSRPLoad, LOW);
  delayMicroseconds(PULSE_WIDTH_USEC);
  digitalWrite(pinBtnSRPLoad, HIGH);
  digitalWrite(pinBtnSREn, LOW);

  //Read button states from shift register
  
  for(int i = 0; i < 8; i++) {
    
        bool bitVal = digitalRead(pinSRData);

        if (bitVal){
          pressedBtns.btnPressed = true;
        }

        switch(i){
          case 0:
            pressedBtns.btn1 = bitVal;
          case 1: 
            pressedBtns.btn2 = bitVal;
          case 2:
            pressedBtns.btn3 = bitVal;
          case 3:
            pressedBtns.btn4 = bitVal;
          case 4:
            pressedBtns.btn5 = bitVal;
          case 5:
            pressedBtns.btn6 = bitVal;
          case 6:
            pressedBtns.btn7 = bitVal;
          case 7:
            pressedBtns.btn8 = bitVal; 
        }


        //Pulse clock to shift next bit out
        digitalWrite(pinSRClock, HIGH);
        delayMicroseconds(PULSE_WIDTH_USEC);
        digitalWrite(pinSRClock, LOW);
    }

  pinMode(pinSRData,OUTPUT);
  
  return pressedBtns;
  
}

void setup() {

  //Set pin Modes
  pinMode(pinSRClock,OUTPUT);
  pinMode(pinSRData,OUTPUT);
  pinMode(pinBtnSREn,OUTPUT);
  pinMode(pinBtnSRPLoad, OUTPUT);

  //StartUp LCD
  lcd.begin(16,2);
  lcd.setLED2Pin(HIGH);

  //Init the SR for Relay and Buzzer
  digitalWrite(pinAuxSREn, LOW);
  shiftOut(pinSRData , pinSRClock, MSBFIRST, B00001111);    
  digitalWrite(pinAuxSREn, HIGH); 

  // Print a message to the LCD.
  sprintf(state.lcdLine1,"Brunet Power LLC");
  sprintf(state.lcdLine2,"2018        V1.0");
  lcdUpdate();
  delay(opts.mainDelay);

  //////////////////////////////////////////
  // Start Self Test
  //////////////////////////////////////////
  sprintf(state.lcdLine1,"Begin           ");
  sprintf(state.lcdLine2,"Self Testing    ");
  lcdUpdate();
  delay(opts.mainDelay);
  

}


void loop() {
 
  lcdUpdate();

  btnState buttons = readButtons();
  
  if(buttons.btnPressed){

    //Reset key press timer
    state.keyPressTimer = 0;

    
    if(buttons.btn3 == 1){
      state.currentScreen = state.currentScreen+1;
    }
    if(buttons.btn2 == 1){
      state.currentScreen = state.currentScreen-1;
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
      sprintf(state.lcdLine2,"2018-01-01      ");
    break;
    
  }
  
  
  
}




long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}



