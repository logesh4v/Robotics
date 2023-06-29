#include <PID_v1.h>



// Motor control pins
int motorPin1 = 3; // IN1
int motorPin2 = 5; // IN2
int motorPin3 = 6; // IN3
int motorPin4 = 9; // IN4

// Motor speeds
int motorSpeed1 = 0;
int motorSpeed2 = 0;

// IR sensor pins
int sensorPin1 = A0;
int sensorPin2 = A1;
int sensorPin3 = A2;
int sensorPin4 = A3;
int sensorPin5 = A4;
int sensorPin6 = A5;
int sensorPin7 = A6;
int sensorPin8 = A7;

// PID constants
double kp = 2; // Proportional constant
double ki = 0.5; // Integral constant
double kd = 0.1; // Derivative constant

// Setpoint and output variables
double setpoint = 512; // Middle value of the sensor range
double output = 0;


double sensor1 = 0;
double sensor2 = 0;

// PID objects
PID pid1(&sensor1, &output, &setpoint, kp, ki, kd, DIRECT);
PID pid2(&sensor2, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  // Initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  // Initialize sensor pins
  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(sensorPin4, INPUT);
  pinMode(sensorPin5, INPUT);
  pinMode(sensorPin6, INPUT);
  pinMode(sensorPin7, INPUT);
  pinMode(sensorPin8, INPUT);

  // Set PID tuning parameters
  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);
}

void loop() {
  // Read sensor values
  int sensorValue1 = analogRead(sensorPin1);
  int sensorValue2 = analogRead(sensorPin2);
  int sensorValue3 = analogRead(sensorPin3);
  int sensorValue4 = analogRead(sensorPin4);
  int sensorValue5 = analogRead(sensorPin5);
  int sensorValue6 = analogRead(sensorPin6);
  int sensorValue7 = analogRead(sensorPin7);
  int sensorValue8 = analogRead(sensorPin8);

  // Compute weighted average of sensor values
  double sensor1 = (sensorValue1 * 1 + sensorValue2 * 2 + sensorValue3 * 3 + sensorValue4 * 4 + sensorValue5 * 5 + sensorValue6 * 6 + sensorValue7 * 7 + sensorValue8 * 8) / (1 + 2 + 3 + 4 + 5 + 6 + 7 + 8);
  double sensor2 = (sensorValue1 * -8 + sensorValue2 * -7 + sensorValue3 * -6 + sensorValue4 * -5 + sensorValue5 * -4 + sensorValue6 * -3 + sensorValue7 * -2 + sensorValue8 * -1) / (1 + 2 + 3 + 4 + 5 + 6 + 7 + 8);

  // Compute PID output
  pid1.Compute();
  pid2.Compute();

  // Apply motor control
  motorSpeed1 = output;
  motorSpeed2 = -output;

  if (motorSpeed1 > 255) 
  {
    motorSpeed1 = 255;
  }
}
