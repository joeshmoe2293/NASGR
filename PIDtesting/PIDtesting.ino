#include <PID_v1.h>

int left = 12;
int right = 13;
int leftInputStream = 1;
int rightInputStream = 2;

double targetLeft;
double targetRight;

double inputLeft;
double inputRight;

double outputLeft;
double outputRight;

double kp=1, ki=0.05, kd=0.25;

PID leftPID(&inputLeft, &outputLeft, &targetLeft, kp, ki, kd, DIRECT);
PID rightPID(&inputRight, &outputRight, &targetRight, kp, ki, kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  pinMode(leftInputStream, INPUT);
  pinMode(rightInputStream, INPUT);

  inputLeft = analogRead(leftInputStream);
  inputRight = analogRead(rightInputStream);

  targetLeft = 100;
  targetRight = 100;

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  inputLeft = analogRead(leftInputStream);
  inputRight = analogRead(rightInputStream);

  leftPID.Compute();
  rightPID.Compute();

  analogWrite(left, outputLeft);
  analogWrite(right, outputRight);
}
