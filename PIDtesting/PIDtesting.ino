#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h>

int leftMotor = 12;
int rightMotor = 13;

double arrayOfInputs[4]; // [0, 1] are joystick X and Y, [2, 3] are measured rotation values from the motors

double outputLeft;
double outputRight;

double kp=1, ki=0.05, kd=0.25;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

PID leftPID(&arrayOfInputs[2], &outputLeft, &arrayOfInputs[0], kp, ki, kd, DIRECT);
PID rightPID(&arrayOfInputs[3], &outputRight, &arrayOfInputs[1], kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Serial.println("Orientation Sensor test\n");

  if (!bno.begin()) {
    Serial.println("Didn't detect sensor... check your connections");
  } else {
    bno.setExtCrystalUse(true);
  }
  
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
}

void loop() {
  for (int i = 0; i < 10; i++) {
    printSensorInformation();
    runPIDs();
    delay(100);
  }
  while(1); // End of program effectively
}

void printSensorInformation() {
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print("Sensor orientation X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tSensor orientation Y: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tSensor orientation Z: ");
  Serial.print(event.orientation.z, 4);
}

void runPIDs() {
  readInputs();
  
  leftPID.Compute();
  rightPID.Compute();

  analogWrite(leftMotor, outputLeft);
  analogWrite(rightMotor, outputRight);
}

// Replace later
void readInputs() {
  for (int i = 0; i < 4; i++) {
    arrayOfInputs[i] = random(90, 100);
  }
}
