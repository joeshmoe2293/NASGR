#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h>

// Pin Variables
int leftMotor = 12;
int rightMotor = 13;

// For demo
constexpr int TOTAL_TIME_TO_RUN = 1000; // In milliseconds

// Array to simulate getting inputs
double arrayOfInputs[4]; // [0, 1] are (calculated elsewhere, maybe from image processing) left and right input values, [2, 3] are measured rotation values from the motors

// Output values for each motor
double outputLeft;
double outputRight;

// Values for PIDs
double kp = 1, ki = 0.05, kd = 0.25;

// The gyroscope, need to figure out what the parameter means...
Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool IMU_working = false; // Used so that we don't try to print out nonexistent sensor data if we're not properly hooked up

// The PIDs!
PID leftPID(&arrayOfInputs[2], &outputLeft, &arrayOfInputs[0], kp, ki, kd, DIRECT);
PID rightPID(&arrayOfInputs[3], &outputRight, &arrayOfInputs[1], kp, ki, kd, DIRECT);

void setup() {
  // Start serial
  Serial.begin(9600);

  // Test connection of IMU
  Serial.println("Orientation Sensor test\n");
  if (!bno.begin()) {
    Serial.println("Didn't detect sensor... check your connections");
  } else {
    bno.setExtCrystalUse(true);
    IMU_working = true;
  }

  // Set the motor pins to output
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);

  // Let the PIDs control the outputLeft and outputRight, MANUAL would make it so we set the values of the outputLeft and outputRight without letting the PID fiddle with them
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
}

void loop() {
  // This is just for the demo, will move the robot for the TOTAL_TIME_TO_RUN in a straight-ish line
  for (int i = 0; i < TOTAL_TIME_TO_RUN/100; i++) {
    printSensorInformation();
    runMotors();
    delay(TOTAL_TIME_TO_RUN/10);
  }
  while(1); // End of program for now
}

// Prints out what we read from the IMU if we're connected
void printSensorInformation() {
  if (IMU_working) {
    // Get the IMU data
    sensors_event_t event;
    bno.getEvent(&event);

    // Print out the IMU data
    Serial.print("Sensor orientation X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tSensor orientation Y: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tSensor orientation Z: ");
    Serial.print(event.orientation.z, 4);
    Serial.println();
  }
}

// Updates the PIDs and runs the motors
void runMotors() {
  readInputs();

  // Update the PIDs
  leftPID.Compute();
  rightPID.Compute();

  // Output to motors
  analogWrite(leftMotor, outputLeft);
  analogWrite(rightMotor, outputRight);
}

// For now, just generates random values for the joystick inputs and the encoder values that are at about 90% full forward
void readInputs() {
  for (int i = 0; i < 2; i++) {
    arrayOfInputs[i] = 1900;
  }
  for (int i = 2; i < 4; i++) {
    arrayOfInputs[i] = static_cast<double>(random(1860, 1900));
  }
}
