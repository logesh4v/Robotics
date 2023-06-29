#include <PID_v1.h>

// Motor pins
#define LEFT_MOTOR_FORWARD 3
#define LEFT_MOTOR_BACKWARD 5
#define RIGHT_MOTOR_FORWARD 6
#define RIGHT_MOTOR_BACKWARD 9

// IR sensor pins
#define SENSOR_1 A0
#define SENSOR_2 A1
#define SENSOR_3 A2
#define SENSOR_4 A3
#define SENSOR_5 A4
#define SENSOR_6 A5
#define SENSOR_7 A6
#define SENSOR_8 A7

// PID constants
double kp = 1.5;
double ki = 0.5;
double kd = 0.5;

// PID variables
double input, output, setpoint;
PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  // Motor pins as outputs
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // IR sensor pins as inputs
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SENSOR_5, INPUT);
  pinMode(SENSOR_6, INPUT);
  pinMode(SENSOR_7, INPUT);
  pinMode(SENSOR_8, INPUT);

  // Set the setpoint to the middle of the sensors
  setpoint = 127;

  // Set the PID parameters
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255, 255);
}

void loop() {
  // Read the sensor values
  int sensor1 = analogRead(SENSOR_1);
  int sensor2 = analogRead(SENSOR_2);
  int sensor3 = analogRead(SENSOR_3);
  int sensor4 = analogRead(SENSOR_4);
  int sensor5 = analogRead(SENSOR_5);
  int sensor6 = analogRead(SENSOR_6);
  int sensor7 = analogRead(SENSOR_7);
  int sensor8 = analogRead(SENSOR_8);

  // Calculate the error
  input = sensor1 * 1 + sensor2 * 2 + sensor3 * 3 + sensor4 * 4 + sensor5 * 5 + sensor6 * 6 + sensor7 * 7 + sensor8 * 8;
  input = input / (sensor1 + sensor2 + sensor3 + sensor4 + sensor5 + sensor6 + sensor7 + sensor8);
  int error = input - setpoint;

  // Calculate the output using PID
  pid.Compute();

  // Set the motor speeds
  int left_speed = constrain(255 + output, 0, 255);
  int right_speed = constrain(255 - output, 0, 255);
  analogWrite(LEFT_MOTOR_FORWARD, left_speed);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  analogWrite(RIGHT_MOTOR_FORWARD, right_speed);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}
