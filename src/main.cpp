#include <Arduino.h>

#define Motor_A1 11 //11
#define Motor_A2 10 //10
#define Motor_B1 9  //9
#define Motor_B2 5  //5
#define rot_R1 2 // R
#define rot_R2 3 // L

void rotate_l(); // because of platform IO
void rotate_r();
void forward(int speed);
void stop_();
void backwards(int speed);
void turn_l(int speed);
void turn_r(int speed);
void updaterotation_R2();
void updaterotation_R1();

void setup() {
  Serial.begin(9600);
  pinMode(Motor_A1, OUTPUT);
  pinMode(Motor_A2, OUTPUT);
  pinMode(Motor_B1, OUTPUT);
  pinMode(Motor_B2, OUTPUT);
  pinMode(rot_R1, INPUT_PULLUP);
  pinMode(rot_R2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rot_R1), updaterotation_R1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rot_R2), updaterotation_R2, CHANGE);
}

void loop() {
  
  /*
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
   */
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

void updaterotation_R1()
{
  //static unsinged, noInterrupts(), interrupts(), 
}
void updaterotation_R2()
{
  Serial.println("2");
}