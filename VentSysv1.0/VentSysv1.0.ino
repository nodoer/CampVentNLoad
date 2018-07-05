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
AuxState axState;
BtnState buttons;

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

  //StartUp LCD
  lcd.begin(16,2);
  lcd.setLED1Pin(HIGH);
  lcd.setLED2Pin(HIGH);

  //Init the SR for Relay and Buzzer
  digitalWrite(pinAuxSREn, LOW);
  shiftOut(pinSRDataOut , pinSRClock, MSBFIRST, B00001111);    
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

  readButtons();

  auxUpdate();

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
      sprintf(state.lcdLine2,"2018-01-01      ");
    break;

    case timeScreen:
      sprintf(state.lcdLine1,"Time            ");
      sprintf(state.lcdLine2,"19:01:01        ");
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



