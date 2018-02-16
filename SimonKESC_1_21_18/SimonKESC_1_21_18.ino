#include <Servo.h>
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int esc3Pin = 3;
int esc5Pin = 5;
int esc6Pin = 6;
int esc9Pin = 9;
int minPulseRate = 1000;
int maxPulseRate = 2000;
int throttleChangeDelay = 100;
int inByte = 0;
int in1 = 10;
int in2 = 11;
int in3 = 8;


void setup() {
  delay(3000);
  Serial.begin(9600);
  Serial.setTimeout(500);

  // Attach the the servo to the correct pin and set the pulse range
  esc1.attach(esc3Pin, minPulseRate, maxPulseRate);
  esc2.attach(esc5Pin, minPulseRate, maxPulseRate);
  esc3.attach(esc6Pin, minPulseRate, maxPulseRate);
  esc4.attach(esc9Pin, minPulseRate, maxPulseRate);

  // Write a minimum value (most ESCs require this correct startup)
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);
}

/*   MOTOR LAYOUT:
      [2]-----[1]
       |       |
       |       |
       |       |
      [3]-----[4]
*/

void loop() {
  //read my bluetooth
  if (Serial.available() > 0) {
    inByte = Serial.read();
    Serial.print("I received: ");
    Serial.println(inByte);

    switch (inByte) {
      case 'W': {//FORWARD
          // Read the new throttle value
          int throttle = 100;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle - 20);
          esc2.write(throttle);
          esc3.write(throttle);
          esc4.write(throttle - 20);
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);

        }
        break;

      case 'S': { //REVERSE
          // Read the new throttle value
          int throttle = 100;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle);
          esc2.write(throttle - 20);
          esc3.write(throttle - 20);
          esc4.write(throttle);
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
        }
        break;

      case 'A': { //STRAFE LEFT
          // Read the new throttle value
          int throttle = 100;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle - 20);
          esc2.write(throttle - 20);
          esc3.write(throttle);
          esc4.write(throttle);
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
        }
        break;

      case 'D': { //STRAFE RIGHT
          // Read the new throttle value
          int throttle = 100;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle);
          esc2.write(throttle);
          esc3.write(throttle - 20);
          esc4.write(throttle - 20);
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
        }
        break;


      case 'R': { //SINK
          // Read the new throttle value
          int throttle = 90;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle);
          esc2.write(throttle);
          esc3.write(throttle);
          esc4.write(throttle);
          digitalWrite(in1, HIGH);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
        }
        break;

      case 'Q': { //TURN LEFT
          // Read the new throttle value
          int throttle = 90;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle);
          esc2.write(throttle+20);
          esc3.write(throttle);
          esc4.write(throttle+20);
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
        }
        break;
        
      case 'E': { //TURN RIGHT
          // Read the new throttle value
          int throttle = 90;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle-12);
          esc2.write(throttle);
          esc3.write(throttle-12);
          esc4.write(throttle);
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
        }
        break;
        
              case 'X': { //STOP
          // Read the new throttle value
          int throttle = 90;
          // Print it out
          Serial.print("Setting throttle to: ");
          Serial.println(throttle);
          esc1.write(throttle);
          esc2.write(throttle);
          esc3.write(throttle);
          esc4.write(throttle);
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
        }
        break;

    }
  }
}

