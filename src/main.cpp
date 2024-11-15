#include <Arduino.h>

#define Motor_A1 11 //11
#define Motor_A2 10 //10
#define Motor_B1 9  //9
#define Motor_B2 3  //3

void rotate_l(int speed);
void forward(int speed);
void stop_();
void backwards(int speed);
void rotate_r(int speed);
void turn_l(int speed);
void turn_r(int speed);

void setup() {
  // put your setup code here, to run once:
  pinMode(Motor_A1, OUTPUT);
  pinMode(Motor_A2, OUTPUT);
  pinMode(Motor_B1, OUTPUT);
  pinMode(Motor_B2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // forward(255);
  // delay(2000);
  // backwards(255);
  // delay(2000);
  // rotate_r(200);
  // delay(1000);
  // rotate_l(200);
  // delay(1000);
  // turn_l(200);
  // delay(1500);
  // turn_r(200);
  // delay(2000);
  // stop_();
  // delay(5000);
}

void forward(int speed) {
  analogWrite(Motor_A1, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_A2, 255);
  analogWrite(Motor_B2, 255);
  delay(10);
  analogWrite(Motor_A2, speed);
  analogWrite(Motor_B2, speed);
}
void stop_() {
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B2, 0);
  analogWrite(Motor_A1, 0);
  analogWrite(Motor_B1, 0);
}

void backwards(int speed) { // done
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B2, 0);
  analogWrite(Motor_A1, 255);
  analogWrite(Motor_B1, 255);
  delay(10);
  analogWrite(Motor_A1, speed * 0.95);
  analogWrite(Motor_B1, speed);
}

void turn_r(int speed) {
  analogWrite(Motor_A1, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_A2, speed);
  analogWrite(Motor_B2, speed * 0.5);
}

void turn_l(int speed) {
  analogWrite(Motor_A1, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_A2, speed*0.5);
  analogWrite(Motor_B2, speed);
}

void rotate_r(int speed) {
  analogWrite(Motor_A1, 0);
  analogWrite(Motor_B2, 0);
  analogWrite(Motor_A2, speed);
  analogWrite(Motor_B1, speed);
}

void rotate_l(int speed) {
  analogWrite(Motor_A2, 0);
  analogWrite(Motor_B1, 0);
  analogWrite(Motor_A1, speed);
  analogWrite(Motor_B2, speed);
}