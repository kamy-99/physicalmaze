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

//const int trigPin2 = ;
//const int echoPin = ;
float ver_dis2;

// because of platform IO
void rotate_l();
void rotate_r();
void forward(int distance);
void backwards(int distance);
void stop_();
void turn_l();
void turn_r();
void updaterotation_R2();
void updaterotation_R1();
void updateSensor1();
void updateSensor2();
void sonar1();
void sonar2();

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
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
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
  if (there's the finish) { priority #1
    put down item
    back off
    send done signal
    stop
  } else {
    if (on the right >5cm (-> there's a path)) { priority #2
      rotate right
      go forward X cm  
    } else if (nothing forward) { priority # 3
      go forward
    } else if (wall in front && wall in right) {
      rotate left
    } else {
      go forward
    }
  }
  */
  updateSensor1();
  updateSensor2();
}

void forward(int distance) { // should be done
  stop_();
  int rotations = round(40 * ((distance / WheelC) * 100));

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= rotations && LRotation <= rotations) {
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_A1, 50);
    analogWrite(MOTOR_B1, 25);
  }
  stop_();
}

void stop_() {
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
}

void backwards(int distance) { // should be done
  stop_();
  int rotations = round(40 * ((distance / WheelC) * 100));

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= rotations && LRotation <= rotations) {
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B2, LOW);
    analogWrite(MOTOR_A1, 222);
    analogWrite(MOTOR_B1, 255);
  }
  stop_();
}

void turn_r() {
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (LRotation < 2 && RRotation < 1) {
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_A1, 50);
    analogWrite(MOTOR_B1, 25);
  }
  stop_();
}

void turn_l() { // changed it to turn a very small amount jut for corrections
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (LRotation < 1 && RRotation < 2) {
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_A1, 50);
    analogWrite(MOTOR_B1, 25);
  }
  stop_();
}

void rotate_r() { // should be done
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

void rotate_l() { // should be done
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= 15 && LRotation <= 15) {
    analogWrite(MOTOR_A1, 255);
    digitalWrite(MOTOR_B2, HIGH); // LEFT GOES BACK

    analogWrite(MOTOR_B1, 25);
    digitalWrite(MOTOR_A2, LOW);
  }
  analogWrite(MOTOR_A1, 25);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(MOTOR_B1, 255);
  digitalWrite(MOTOR_A2, HIGH);
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

void updateSensor1() { // right sonar (this gets n1 because it's the bigger priority)
  static unsigned long timer;
  if (millis() > timer) {
    sonar1();
    Serial.println("Sonar1 update");
    timer = millis() + 250; // update every 0.25s can change
  }
  
}

void updateSensor2() { // forward sonar
  static unsigned long timer = 0;
  if (millis() > timer) {
    sonar2();
    Serial.println("Sonar2 update");
    timer = millis() + 250; // update every 0.25s can change
  }
}

void sonar2() // forward sonar
{
  float distance[2] = {0.0, 0.0};
  float duration;
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
    Serial.print("Distance: ");
    Serial.println(ver_dis);
}

void sonar1() // right sonar
{
  float distance[2] = {0.0, 0.0};
  float duration;
    for(int i = 0; i < 2; i++)
    {
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);

    duration = pulseIn(echoPin2, HIGH);
    distance[i] = (duration*.0343)/2;
    }
    if(distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) <= 5)
    {
      ver_dis2 = distance[1];
    }
    Serial.print("Distance: ");
    Serial.println(ver_dis2);
}