#include <Arduino.h>

#define Motor_A1 11 //11
#define Motor_A2 10 //10
#define Motor_B1 9  //9
#define Motor_B2 3  //3

void rotate_l();
void rotate_r();
void forward(int speed);
void stop_();
void backwards(int speed);
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
   forward(255);
   delay(3000);
   stop_();
   delay(3000);

   backwards(255);
   delay(3000);
   stop_();
   delay(3000);

   turn_l(255);
   stop_();
   delay(3000);

   turn_r(255);
   stop_();
   delay(3000);

   rotate_r();
   stop_();
   delay(3000);
  
   rotate_l();
   stop_();
   delay(3000);
}

void forward(int speed) {
  stop_();
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
  stop_();
  analogWrite(Motor_A1, 255);
  analogWrite(Motor_B1, 255);
  delay(10);
  analogWrite(Motor_A1, speed * 0.98);
  analogWrite(Motor_B1, speed);
}

void turn_r(int speed) {
  stop_();
  analogWrite(Motor_A2, 255);
  analogWrite(Motor_B2, 255);
  delay(10);
  analogWrite(Motor_A2, speed);
  analogWrite(Motor_B2, speed * 0.8);
  delay(8000);
}

void turn_l(int speed) {
  stop_();
  analogWrite(Motor_A2, 255);
  analogWrite(Motor_B2, 255);
  delay(10);
  analogWrite(Motor_A2, speed*0.7);
  analogWrite(Motor_B2, speed);
  delay(4000);
}

void rotate_r() {
  stop_();
  for (int i = 0; i < 55; i++) {
    analogWrite(Motor_B1, 170);
    analogWrite(Motor_A2, 255);
    delay(10);
    analogWrite(Motor_A2, 170);
    delay(1);
  }
  analogWrite(Motor_B2, 0);
  analogWrite(Motor_A1, 0);
}

void rotate_l() {
  stop_();
  for (int i = 0; i < 55; i++) {
    analogWrite(Motor_B2, 170);
    analogWrite(Motor_A1, 255);
    delay(10);
    analogWrite(Motor_A1, 170);
    delay(1);
  }
  analogWrite(Motor_B2, 0);
  analogWrite(Motor_A1, 0);
}