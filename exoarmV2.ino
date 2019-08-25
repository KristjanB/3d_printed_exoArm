#include <HX711_ADC.h>
#include <EEPROM.h>

//HX711 constructor (dout pin, sck pin):
HX711_ADC LoadCell(2, 5);

int motor_in1 = 3;
int motor_in2 = 4;
int motor_pwm = 6;

float calValue = 107.54;

int motor_max_speed = 255;
float weight = 0;
float torque = 0;
float handle_length = 0.22;
float writemotor;
float deadband = 20;
float angle = 0;
int upper_angle_limit = 8, lower_angle_limit = 150;

void move_up(float writemotor) {
  writemotor = constrain(writemotor, 0 , motor_max_speed);
  digitalWrite(motor_in1, HIGH);
  digitalWrite(motor_in2, LOW);
  analogWrite(motor_pwm, writemotor);
}

void stop_move(void) {
  digitalWrite(motor_in1, LOW);
  digitalWrite(motor_in2, LOW);
  analogWrite(motor_pwm, 0);
}

void move_down(float writemotor) {
  writemotor = constrain(writemotor, 0 , motor_max_speed);
  digitalWrite(motor_in1, LOW);
  digitalWrite(motor_in2, HIGH);
  analogWrite(motor_pwm, writemotor);
}

void jerkoff_mode(float writemotor, int angle) {
  if (angle > 45) move_up(writemotor);
  if (angle < 180) move_down(writemotor);
}




float computePID_angle(float setpoint_angle, float angle) {
  float errorSum;
  float Kp = 0.5;
  float Ki = 0.01;
  float dtrate, last;

  double time_elapsed = micros();
  dtrate = (time_elapsed - last) / 1000.f;
  last = time_elapsed;

  float error = setpoint_angle - angle;
  errorSum += error * dtrate;
  errorSum = constrain(errorSum, -255, 255);
  //Serial.println(error);


  return (Kp * error) + (Ki * errorSum);
}


float computePID_torque(float setpoint_torque, float torque) {
  float errorSum;
  float Kp = 0.3;
  float Ki = 0.03;
  float dtrate, last;

  double time_elapsed = micros();
  dtrate = (time_elapsed - last) / 1000.f;
  last = time_elapsed;

  float error = setpoint_torque - torque;
  errorSum += error * dtrate;
  errorSum = constrain(errorSum, -255, 255);
  //Serial.println(error);


  return (Kp * error) + (Ki * errorSum);
}


int measured_torque = 0;
int measured_weight = 0;

void setup() {

  Serial.begin(9600); delay(10);
  LoadCell.begin();
  long stabilisingtime = 1000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  if (LoadCell.getTareTimeoutFlag()) {
    //TODO: signal error
  }
  else {
    LoadCell.setCalFactor(calValue); // set calibration value (float)
  }
  attachInterrupt(digitalPinToInterrupt(2), whenreadyISR, FALLING);

//  while (!Serial.available()) {
//    Serial.println("press a key to start...");
//    if (Serial.available() > 0) {
//      measured_weight = LoadCell.getData();
//      measured_torque = handle_length * measured_weight;
//      Serial.print("measured weight: ");
//      Serial.println(measured_weight);
//      Serial.print("measured torque: ");
//      Serial.println(measured_torque);
//    }
//    //measured_torque = Serial.parseInt();
//  }
  //Serial.println(angle_setpoint_serial);
}



void whenreadyISR() {
  LoadCell.update();
}


float desired_angle = 0;
float desired_torque = 0;
float maxtorque = 100, mintorque = -100;
int steps[] = {50, 60, 70, 80 , 90, 100, 110, 120, 130, 140, 150, 160, 170, 180};

void loop() {


  angle = analogRead(A0);
  angle = map(angle, 131, 978, 45, 180); // 370 = 90 degrees
  weight = LoadCell.getData();
  torque = handle_length * weight;

  //  if (torque > maxtorque) maxtorque = torque;
  //  if (torque <= mintorque) mintorque = torque;
  //
  //  writemotor = map(torque, -255, 255, -255, 255);


  desired_torque = computePID_torque(0, torque);
  writemotor = -1 * desired_torque; //map(desired_torque, -250 , 250, 255, -255);

  //  Serial.print("desired_angle: ");
    Serial.println(torque);
  //  Serial.print("  ");
  //Serial.println(writemotor);



  //*** MOVE DOWN ***//
  if (writemotor > deadband) move_down(writemotor);      // positive values of writemotor

  //*** MOVE UP ***//
  if (writemotor < -deadband) move_up(-1 * writemotor);

  //*** STOP MOVEMENT ***//
  if (writemotor >= -deadband && writemotor <= deadband) stop_move();



}
