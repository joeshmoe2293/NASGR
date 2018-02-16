#include <Servo.h>

// For shortening any bitmask creation
typedef const unsigned char bitmask;

// Container class for the ESCs on the robot (for making code cleaner)
class ESCArray {
  // Contains an array of ESC servos
  private:
    static const uint8_t _numControllers = 4;
    Servo _controllers[_numControllers];
    static const int minPulseRate = 1000;
    static const int maxPulseRate = 2000;

  public:
    ESCArray() {}

    // For creating and attaching servos to the input pins
    void generateESCs(const uint8_t* pins) {
      for (uint8_t i = 0; i < _numControllers; i++) {
        Servo servo;
        servo.attach(pins[i], minPulseRate, maxPulseRate);
        _controllers[i] = servo;
      }
    }

    // For writing a throttle power to motors
    void write(const uint8_t throttle, const uint8_t = 0, bitmask = 0);
};

// Writes a power of throttle to all motors, and adds offset to those that are contained in subtractFrom
void ESCArray::write(const uint8_t throttle, const uint8_t offset, bitmask subtractFrom) {
  for (uint8_t i = 0; i < _numControllers; i++) {
    uint8_t power;
    if (subtractFrom & (0x1 << i)) {
      power = throttle + offset;
    } else {
      power = throttle;
    }    
    _controllers[i].write(power);
    Serial.println("Writing power " + String(power) + " to motor " + String(i + 1));
  }
}

// Class for writing powers to pumps more easily
class VerticalThrusterArray {
  // Contains an array of pump pins that it will write HIGH or LOW to
  private:
    static const uint8_t _numThrusters = 2;
    Servo _thrusters[_numThrusters];
    static const int minPulseRate = 1000;
    static const int maxPulseRate = 2000;
    static const uint8_t sinkPower = 70;
    static const uint8_t neutralPower = 90;

  public:
    VerticalThrusterArray() {}

    // Set the pins for the pumps
    void setPins(const uint8_t* pins) {
      for (uint8_t i = 0; i < _numThrusters; i++) {
        Servo servo;
        servo.attach(pins[i], minPulseRate, maxPulseRate);
        _thrusters[i] = servo;
      }
    }

    // Sinks the sub if sink is true
    void write(const bool sink) {
      for (uint8_t i = 0; i < _numThrusters; i++) {
        uint8_t power = sink ? sinkPower:neutralPower;
        _thrusters[i].write(power);
        if (power == sinkPower) {
          Serial.println("Sinking!");
        }
      }
    }
};

// Class that contains ESCs and pumps
class Robot {
  private:
    ESCArray _ESCs;
    VerticalThrusterArray _thrusters;

  public:
    Robot() {}

    // Initializes its array of ESCs and its pumps
    void init(const uint8_t* ESCPins, const uint8_t* thrusterPins) {
      _ESCs.generateESCs(ESCPins);
      _thrusters.setPins(thrusterPins);
    }

    // For sinking the sub, throttle is 90 in the code (for keeping it still while it sinks)
    void sink(const uint8_t throttle) {
      _ESCs.write(throttle);
      _thrusters.write(true);
    }

    // For writing a power to each of the motors it has
    void write(const uint8_t throttle, const uint8_t = 0, bitmask = 0);
};

// Writes to each of the ESCs and tells the sub to not sink
void Robot::write(const uint8_t throttle, const uint8_t offset, bitmask mask) {
  _ESCs.write(throttle, offset, mask);
  _thrusters.write(false);
}

// For printing out the throttle for debugging
void printThrottle(uint8_t throttle) {
  Serial.println("Setting throttle to: " + String(throttle));
}

// Not sure what throttleChangeDelay is for, but kept it just in case you have plans for it
uint8_t throttleChangeDelay = 100;
byte inByte;

// All of the ESC and pump info is contained inside this 'robit'
Robot robit;

void setup() {
  delay(3000);
  Serial.begin(9600);
  Serial.setTimeout(500);

  // Make a list of ESC and pump pins, and initialize the robot with said pins
  const uint8_t ESCPins[] = {3, 5, 6, 9};
  const uint8_t thrusterPins[] = {10, 11, 8};
  robit.init(ESCPins, thrusterPins);
  robit.write(0);
}

/*   MOTOR LAYOUT:
      [2]-----[1]
       |       |
       |       |
       |       |
      [3]-----[4]
*/

bitmask forward = 0b1001; // For motors 1 and 4
bitmask reverse = 0b0110; // For motors 2 and 3
bitmask strafeLeft = 0b0011; // For motors 1 and 2
bitmask strafeRight = 0b1100; // For motors 3 and 4
bitmask turnLeft = 0b1010; // For motors 2 and 4
bitmask turnRight = 0b0101; // For motors 1 and 3

void loop() {
  //read my bluetooth
  if (Serial.available() > 0) {
    inByte = Serial.read();
    Serial.print("I received: ");
    Serial.println(inByte);
    uint8_t throttle = 100;
    
    switch (inByte) {
      case 'W': {//FORWARD
        robit.write(throttle, -20, forward);
      }
      break;

      case 'S': { //REVERSE
        robit.write(throttle, -20, reverse);
      }
      break;

      case 'A': { //STRAFE LEFT
        robit.write(throttle, -20, strafeLeft);
      }
      break;

      case 'D': { //STRAFE RIGHT
        robit.write(throttle, -20, strafeRight);
      }
      break;

      case 'R': { //SINK
        throttle = 90;
        robit.sink(throttle);
      }
      break;

      case 'Q': { //TURN LEFT
        throttle = 90;
        robit.write(throttle, 20, turnLeft);
      }
      break;
        
      case 'E': { //TURN RIGHT
        throttle = 90;
        robit.write(throttle, -12, turnRight);
      }
      break;
        
      case 'X': { //STOP
        throttle = 90;
        robit.write(throttle);
      }
      break;
    }
    
    printThrottle(throttle);
  }
}

