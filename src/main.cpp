#include <Arduino.h>

#define MOTOR_A1 11 //11
#define MOTOR_A2 10 //10
#define MOTOR_B1 9  //9
#define MOTOR_B2 5  //5
#define ROT_R1 2 // R
#define ROT_R2 3 // L
#define DVALUE 10 // debounce value
const int trigPin = 7;  
const int echoPin = 8; 
float duration, ver_dis;
float distance[];

void rotate_l(); // because of platform IO
void rotate_r();
void forward(int speed);
void stop_();
void backwards(int speed);
void turn_l(int speed);
void turn_r(int speed);
void updaterotation_R2();
void updaterotation_R1();
void sonar();

int RRotation = 0;
int LRotation = 0;

void setup() {
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
  Serial.begin(9600);
}

void loop() {
  sonar();
  

}

void forward(int speed) {
  stop_();
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B2, 255);
  delay(10);
  analogWrite(MOTOR_A2, speed);
  analogWrite(MOTOR_B2, speed);
}
void stop_() {
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
}

void backwards(int speed) { // done
  stop_();
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_B1, 255);
  delay(10);
  analogWrite(MOTOR_A1, speed * 0.98);
  analogWrite(MOTOR_B1, speed);
}

void turn_r(int speed) {
  stop_();
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B2, 255);
  delay(10);
  analogWrite(MOTOR_A2, speed);
  analogWrite(MOTOR_B2, speed * 0.8);
  delay(8000);
}

void turn_l(int speed) {
  stop_();
  analogWrite(MOTOR_A2, 255);
  analogWrite(MOTOR_B2, 255);
  delay(10);
  analogWrite(MOTOR_A2, speed*0.7);
  analogWrite(MOTOR_B2, speed);
  delay(4000);
}

void rotate_r() {
  stop_();
  for (int i = 0; i < 55; i++) {
    analogWrite(MOTOR_B1, 170);
    analogWrite(MOTOR_A2, 255);
    delay(10);
    analogWrite(MOTOR_A2, 170);
    delay(1);
  }
  analogWrite(MOTOR_B2, 0);
  analogWrite(MOTOR_A1, 0);
}

void rotate_l() {
  stop_();
  // should work have to test tho
  analogWrite(MOTOR_A1, 255);
  analogWrite(MOTOR_B2, 255);
  delay(5);
  while (RRotation <= 18 && LRotation <= 18) {
    analogWrite(MOTOR_A1, 170);
    analogWrite(MOTOR_B2, 170);
  }
  RRotation = 0;
  LRotation = 0;
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
