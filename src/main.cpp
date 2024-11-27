#include <Arduino.h>

#define MOTOR_A1 11 //11 B
#define MOTOR_A2 12 //12 F DIGITAL WAS 10
#define MOTOR_B1 9  //9 B
#define MOTOR_B2 4  //4 F DIGITAl LEFT WHEEL 0 - BACK 1 - FORWARD WAS 5
#define ROT_R1 2 // R
#define ROT_R2 3 // L
#define DVALUE 10 // debounce value in ms
#define PIVALUE 3.141592653589793238462643383279502884197 // 39 digits or so

const int trigPin = 7;  
const int echoPin = 8; 
float ver_dis;

// because of platform IO
void rotate_l();
void rotate_r();
void forward(int distance);
void backwards(int distance);
void stop_();
void turn_l(int speed);
void turn_r(int speed);
void updaterotation_R2();
void updaterotation_R1();
void updateSensor1();
void updateSensor2();
void sonar();

int RRotation = 0;
int LRotation = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ROT_R1, INPUT_PULLUP);
  pinMode(ROT_R2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROT_R1), updaterotation_R1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROT_R2), updaterotation_R2, CHANGE);
}

int WheelC = 6.5*PIVALUE; // wheel circumference in cm 2*r*pi ~20.5cm I am not using our 1 library for getting near perfect PI

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

  do
  {
    sonar();
    rotate_l();
  }
  while(ver_dis > 13.00);
  do
  {
    sonar();
    rotate_r();
  }
  while (ver_dis < 13.00);
  
  // if(1 < ver_dis < 8)
  // {
  //   stop_();
  //   delay(2000);
  //   rotate_r();
  //   turn_l(255);
  // }
  // else
  // {
  //   forward(10);
  // }
  //backwards(10);
  //delay(1000);
}

void forward(int distance) { // needs fix
  stop_();
  int rotations = distance / WheelC;

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= rotations && LRotation <= rotations) { // needs fix
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_A1, 50);
    analogWrite(MOTOR_B1, 25);
  }
}

void stop_() {
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
}

void backwards(int distance) { // needs fix
  stop_();
  int rotations = distance / WheelC;

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= rotations && LRotation <= rotations) { // looks right
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B2, LOW);
    analogWrite(MOTOR_A1, 222);
    analogWrite(MOTOR_B1, 255);
    if(RRotation <= 10 && LRotation <= 10)
    {
      stop_();
    }
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

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= 15 && LRotation <= 15) {
    analogWrite(MOTOR_A1, 25);
    digitalWrite(MOTOR_B2, LOW);

    analogWrite(MOTOR_B1, 255);
    digitalWrite(MOTOR_A2, HIGH);
  }
  analogWrite(MOTOR_A1, 255);
  digitalWrite(MOTOR_B2, HIGH);

  analogWrite(MOTOR_B1, 25);
  digitalWrite(MOTOR_A2, LOW);
  delay(10);
  stop_();
}

void rotate_l() { // still needs some testing for perfect 90 degree turn
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= 15 && LRotation <= 15) {
    analogWrite(MOTOR_A1, 255);
    digitalWrite(MOTOR_B2, HIGH); // LEFT GOES BACK

    analogWrite(MOTOR_B1, 25);
    digitalWrite(MOTOR_A2, LOW); // RIGHT GOES BACK
  }
  analogWrite(MOTOR_A1, 25);
  digitalWrite(MOTOR_B2, LOW); // LEFT GOES BACK
  analogWrite(MOTOR_B1, 255);
  digitalWrite(MOTOR_A2, HIGH); // RIGHT GOES BACK
  delay(10);
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
  static unsigned long timer = 0;
  if (millis() - timer > 250) {
    sonar();
  }
  timer = millis(); // update every 0.25s can change
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

void sonar()
{
  float distance[2] = {0.0, 0.0};
  float duration;
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
    delay(250);
  }
}

