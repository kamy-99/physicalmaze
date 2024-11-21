#include <Arduino.h>

#define MOTOR_A1 11 //11
#define MOTOR_A2 10 //10
#define MOTOR_B1 9  //9
#define MOTOR_B2 5  //5
#define ROT_R1 2 // R
#define ROT_R2 3 // L
#define DVALUE 10 // debounce value in ms
#define PI 3.141592653589793238462643383279502884197 // 39 digits or so

const int trigPin = 7;  
const int echoPin = 8; 
float duration, ver_dis;
float distance[2] = {0.0, 0.0};

// because of platform IO
void rotate_l();
void rotate_r();
void forward(int distance, int speed);
void backwards(int distance, int speed);
void stop_();
void turn_l(int speed);
void turn_r(int speed);
void updaterotation_R2();
void updaterotation_R1();
void updateSensor1();
void updateSensor2();
void Sonar2();

int RRotation = 0;
int LRotation = 0;

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(ROT_R1, INPUT_PULLUP);
  pinMode(ROT_R2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROT_R1), updaterotation_R1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROT_R2), updaterotation_R2, CHANGE);
}

int WheelC = 6.5*PI; // wheel circumference in cm 2*r*pi ~20.5cm I am not using our 1 library for getting near perfect PI

void loop() {
  /*
  for now the logic should look something like this - this is in a perfect world
  also this might not work since it might be a bit "choppy" as it moves

  update sensor 1 (looking right)
  update sensor 2 (looking forward)
  if (there's a line) { priority #1
    follow it
  } else {
    if (nothing on right (-> there's a path)) { priority #2
      rotate right
      go forward X cm  
    } else if (nothing forward) { priority # 3
      go forward
    } else if (wall in front && wall in right) {
      rotate left
    }
  }
  */

  rotate_l();
  delay(5000);
}

void forward(int distance, int speed) { // distance in cm & speed, might need to remove the distance part
  stop_();
  int rotations = distance / WheelC;
  RRotation = 0;
  LRotation = 0;
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B2, 255);
  delay(5);
  while (RRotation <= rotations && LRotation <= rotations) {
    analogWrite(MOTOR_A2, speed);
    analogWrite(MOTOR_B2, speed);
  }
}

void stop_() {
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
}

void backwards(int distance, int speed) { // distance in cm & speed, might need to remove the distance part
  stop_();
  int rotations = distance / WheelC;
  RRotation = 0;
  LRotation = 0;
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_B1, 255);
  delay(5);
  while (RRotation <= rotations && LRotation <= rotations) {
    analogWrite(MOTOR_A1, speed * 0.98);
    analogWrite(MOTOR_B1, speed);
  }
}

void turn_r(int speed) { // don't need to change because we are not using turn only rotate
  stop_();
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B2, 255);
  delay(10);
  analogWrite(MOTOR_A2, speed);
  analogWrite(MOTOR_B2, speed * 0.8);
  delay(8000);
}

void turn_l(int speed) { // don't need to change because we are not using turn only rotate
  stop_();
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B2, 255);
  delay(10);
  analogWrite(MOTOR_A2, speed*0.7);
  analogWrite(MOTOR_B2, speed);
  delay(4000);
}

void rotate_r() { // still needs some testing for perfect 90 degree turn
  stop_();
  RRotation = 0;
  LRotation = 0;
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B1, 255);
  delay(5);
  while (RRotation <= 17 && LRotation <= 17) { // 17 is roughly 90 degrees w/ this speed but if we want to make it faster we have to adjust
    analogWrite(MOTOR_A2, 170);
    analogWrite(MOTOR_B1, 170);
  }
  stop_();
}

void rotate_l() { // still needs some testing for perfect 90 degree turn
  stop_();
  RRotation = 0;
  LRotation = 0;
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_B2, 255);
  delay(5);
  while (RRotation <= 17 && LRotation <= 17) { // 17 is roughly 90 degrees w/ this speed but if we want to make it faster we have to adjust
    analogWrite(MOTOR_A1, 170);
    analogWrite(MOTOR_B2, 170);
  }
  stop_();
}

void updaterotation_R1() // right rotation
{
  static unsigned long timer;
  static bool lastState;
  noInterrupts();
  if (millis() > timer) {
    bool state = digitalRead(ROT_R1);
    if (lastState != state) {
      RRotation++;
      lastState = state;
    }
  timer = millis() + DVALUE;
  }
  interrupts();
}

void updaterotation_R2() // left rotation
{
  static unsigned long timer;
  static bool lastState;
  noInterrupts();
  if (millis() > timer) {
    bool state = digitalRead(ROT_R2);
    if (lastState != state) {
      LRotation++;
      lastState = state;
    }
  timer = millis() + DVALUE;
  }
  interrupts();
}

void updateSensor1() {
  static unsigned long timer;
  if (millis() > timer) {
    // update the sensor for looking right
  }
  timer = millis() + 250; // update every 0.25s can change
}

void updateSensor2() {
  static unsigned long timer;
  if (millis() > timer) {
    sonar2();
  }
  timer = millis() + 250; // update every 0.25s can change
}
// void sonar()
// {
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);

//   duration = pulseIn(echoPin, HIGH);
//   distance = (duration*.0343)/2;
//   Serial.print("Distance: ");
//   Serial.println(distance);
//   delay(250);
// }

void sonar2()
{
  for(int attempt = 0; attempt < 3; attempt++)
  {
    for(int i = 0; i < 2; i++)
    {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance[i] = (duration*.0343)/2;
    }
    if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 5)
    {
      ver_dis = distance[1];
    }
    else
    {
      attempt++;
    }
    Serial.print("Distance: ");
    Serial.println(ver_dis);
  }
}
