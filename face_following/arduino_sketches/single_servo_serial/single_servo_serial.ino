// Example 4 - Receive a number as text and convert it to an int
// SOURCE: https://forum.arduino.cc/t/serial-input-basics-updated/382007/3#example-4-receiving-a-single-number-from-the-serial-monitor-2


#include <Servo.h>


const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data

boolean newData = false;

int dataNumber = 0;             // new for this version

Servo servo; 
int pos = 0;


void setup() {
    servo.attach(8);
    servo.write(0);
    Serial.begin(115200);
}

void loop() {
    recvWithEndMarker();
    showNewNumber();
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewNumber() {
    if (newData == true) {
        pos = 0;             // new for this version
        pos = atoi(receivedChars);   // new for this version

        if(pos>0 && pos<180) {
          servo.write(pos);
        }

        newData = false;
    }
}
