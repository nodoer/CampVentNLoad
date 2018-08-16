#include <EEPROM.h>

struct MyObject {
  float field1 = 12.5;
  byte field2 =16;
  char name[10] = "helllother";
};

MyObject options;

void setup() {
    Serial.begin(9600);
    while(!Serial){};

    //If there is data in the EEPROM
    //read it into the options struct
    if(EEPROM.read(0) == 255){
      EEPROM.write(0,0);
      EEPROM.put(1,options);
    } else {
      EEPROM.get(1,options);
    }


      Serial.println("Read custom object from EEPROM: ");
      Serial.println(options.field1);
      Serial.println(options.field2);
      Serial.println(options.name);

      options.field1 = 55.6;

      EEPROM.put(1,options);
    
      //Reset EEPROM by writing 255 to the control byte
      //at address 0
      EEPROM.write(0,255);

}

void loop() {
  // put your main code here, to run repeatedly:

}
