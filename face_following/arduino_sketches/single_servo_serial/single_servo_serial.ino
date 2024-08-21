#include <Servo.h>

Servo servo; 
String inByte;
int pos;

void setup() {
  servo.attach(8);
  servo.write(0);
  Serial.begin(115200);
}

void loop() {    
  if(Serial.available()) { 
    inByte = Serial.readStringUntil('\n');
    pos = inByte.toInt();
    
    Serial.print(pos);
    
    if(pos>=0 && pos<=180) {
      servo.write(pos);
    }
  }
}