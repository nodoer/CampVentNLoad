int RCLKPin = 9;   // pin 12 on the 74hc595 latch - nSS 
int SRCLKPin = 6;  // pin 11 on the 74hc595 shift register clock - SCK
int SERPin = 5;    // pin 14 on the 74hc595 data - MOSI
unsigned int d;    // Data to be sent to the shift reg.
int dir =0;        // Direction of walking 1.
char buf[12];      // General purpose buffer.


void setup() {
  Serial.begin(57600);          // start serial port (debug).
 
  pinMode(RCLKPin, OUTPUT);    // Set 595 control PIN sto output.
  pinMode(SRCLKPin, OUTPUT);
  pinMode(SERPin, OUTPUT);

  Serial.println("74HC595 Demo.");

  d=1;
}


void loop() {
      
    delay(100);
    digitalWrite(RCLKPin, LOW);
    shiftOut(SERPin, SRCLKPin, MSBFIRST, 0x00ff & d);    
    digitalWrite(RCLKPin, HIGH);     
    Serial.println(itoa(d,buf,16));

    if (!dir) d<<=1; else d>>=1; // Shift
    
    if (d&0x80) dir=1;           // Set direction.
    if (d&0x01) dir=0;
}
