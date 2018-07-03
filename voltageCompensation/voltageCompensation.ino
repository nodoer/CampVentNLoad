/*
  ReadAnalogVoltage
  Reads an analog input on pin 0, converts it to voltage, and prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {

  float maxVoltage12 = 20.55;
  float maxVoltage5 = 10.65;

  float railVoltage = readVcc();
  Serial.println(railVoltage/1000);
  float railOffset = (railVoltage/1000) - 3.3;

  Serial.println(railOffset);
  delay(2); //Stabalize Vref
  float v12Factor = (20.5+railOffset) / 3.3;
  float v5Factor = (10.65+railOffset) / 3.3;

  float v12Avg = 0;
  for(int i = 0; i<20; i++){
    v12Avg = v12Avg + analogRead(A0);
  }
  float v12Voltage = (v12Avg/20) * ((railVoltage/1000)*v12Factor / 1023.0);
  Serial.println(v12Voltage);


float v5Avg = 0;
  for(int i = 0; i<20; i++){
    v5Avg = v5Avg + analogRead(A1);
  }
  float v5Voltage = (v5Avg/20) * ((railVoltage/1000)*v5Factor / 1023.0);
  Serial.println(v5Voltage);


  delay(2000);
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
