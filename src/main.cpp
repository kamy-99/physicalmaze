#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define MOTOR_A1 11 //11 B
#define MOTOR_A2 12 //12 F DIGITAL WAS 10
#define MOTOR_B1 9  //9 B
#define MOTOR_B2 4  //4 F DIGITAl LEFT WHEEL 0 - BACK 1 - FORWARD WAS 5
#define ROT_R1 2 // R
#define ROT_R2 3 // L
#define DVALUE 10 // debounce value in ms
#define PIVALUE 3.141592653589793238462643383279502884197 // 39 digits or so
#define GRIPPER 10
#define LEDPIN 13
int NUMPIXELS = 4;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_RGB + NEO_KHZ800);


const int trigPin = 7;  
const int echoPin = 8; 
float forward_dis;

const int trigPin2 = 5;
const int echoPin2 = 6;
float right_dis;

int steeringError = 0;
float sonarWeights[] = {-5, -4, -3, 1, 1, 3, 4, 5};  // A0 has weight -3, A7 has weight +3
// PID variables
float Kp = 7.25;    // Increase proportional constant for quicker response 7.25 || 9 || 8.5
float Ki = 1.25;    // Small integral constant to reduce drift over time 1.25 || 9 || 7.0
float Kd = 0.75;   // Moderate derivative constant to dampen oscillations 0.5 || 0.75 || 2.0

int lastKnownline_SteeringError = 0;
float prevError = 0;    // Previous error value (for derivative term)
float integral = 0;     // Accumulated error (for integral term)

bool flagUp = false;
bool hasExecuted = false;
int SERVO_INTERVAL = 20;

int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Arrays to store sensor values
int minlineValue[8] = {9999};
int maxlineValue[8] = {-9999};
int avglineValue[8];
int High_Threshold[8];
int Low_Threshold[8];


// Sensor states
int sensorStates[8]; // Holds the current state for each sensor
int sensorValues[8];
// Define the weights for each sensor (based on their distance from the center)
int sensorWeights[] = {-7, -5, -3, 1, 1, 3, 5, 7};  // A0 has weight -7, A7 has weight +7

int line_steeringError = 0;

// Add an array to store the previous state of each sensor
int previousSensorStates[8];

#define AVERAGE_WINDOW 1  // Number of readings to average
int recentReadings[8][AVERAGE_WINDOW];
int readingIndex = 0;

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
void gripper(int angle);
void gameLogic();
void failSafe();
void updateKeepDistance();
void keepDistance();
int calculateRotationCount(int degrees);
void preciseRotate_r(int degrees);
void logictest();
void adjustSteering(int steeringError);
int steering_Error();
int PID(int steeringError);
void spin_Left1();
void forwardRotation2();
void forwardRotation1();
void grabCone();
void flagCheck();
void reset_Rotations();
void dropCone();
void line_steerError();
void determineStates();
void lineCalibration();
void calibrateSensors();
void updateLineCalibration();
void Initializing();
void line_adjustSteering(int line_steeringError);
void normal_Pixel();
void braking_Pixel();
void right_Pixel();
void left_Pixel();
void back_Pixel();




int RRotation = 0;
int LRotation = 0;

int c1 = 0;
int c2 = 0;


void setup() {
  Serial.begin(9600);
  Initializing();
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(GRIPPER, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(ROT_R1, INPUT_PULLUP);
  pinMode(ROT_R2, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ROT_R1), updaterotation_R1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROT_R2), updaterotation_R2, CHANGE);
}

float WheelC = 6.5*PIVALUE; // wheel circumference in cm 2*r*pi ~20.5cm I am not using our 1 library for getting near perfect PI

void loop() {
  grabCone();
  lineCalibration();         // Update sensor values
  determineStates();         // Determine black/white states
  line_steerError();         // Calculate steering error

    // Use the steering error to adjust motor speeds
  line_adjustSteering(line_steeringError);


//   gripper(0);
//   steering_Error();
//   flagCheck();
//   grabCone();
//   if(flagUp == true)
//   {
//   line_adjustSteering(line_steeringError);
//   //adjustSteering(steeringError);
//   }
  // steering_Error();
  // flagCheck();
  // grabCone();
  // updateLineCalibration();
  // determineStates();
  // int line_steeringError = 0;
  //   for (int i =0; i < 8; i++)
  //  {
  //   if (sensorStates[i] == "black")
  //   {
  //     line_steeringError += sensorWeights[i];  // Add weighted sensor value to the total error
  //   }
  //  }
  //  line_adjustSteering(line_steeringError);
  //  while (line_steeringError == 0)
  // {
  //   // Apply motor speeds
  //   steering_Error();
  //   adjustSteering(steeringError);
  //   Serial.println("line_steeringError: ");
  //   Serial.print(line_steeringError);      
  // }
  // line_adjustSteering(line_steeringError);
  // if(flagUp == true)
  // {
  // line_adjustSteering(line_steeringError);
  // //adjustSteering(steeringError);
  // }
//     for (int i = 0; i < 8; i++) {
//     Serial.print("Sensor ");
//     Serial.print(i);
//     Serial.print(": ");
//     Serial.println(sensorValues[i]);
// }
}

// updates
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

void updateKeepDistance() { // this will try to keep the distance from the wall consistent 
  static unsigned long timer;
  if (millis() > timer) {
    keepDistance();
    Serial.println("keepDistance update");
    timer = millis() + 250; // update every 0.25s can change could be
  }
}

void updateGameLogic() { // update gamelogic
  static unsigned long timer;
  if (millis() > timer) {
    gameLogic();
    Serial.println("gameLogic update");
    timer = millis() + 250; // update every 0.25s can change could be
  }
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
      c1 = 0;
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
      c2 = 0;
    }
  timer = millis() + DVALUE;
  }
  interrupts();
}

// movement
void forward(int distance) {
  stop_();
  int rotations = round((distance / WheelC) * 20);
  
  noInterrupts(); // Reset rotation counters
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= rotations && LRotation <= rotations) {
    normal_Pixel();
    digitalWrite(MOTOR_A2, HIGH); // Set left motor forward
    digitalWrite(MOTOR_B2, HIGH); // Set right motor forward
    analogWrite(MOTOR_A1, 60);    // Speed for left motor
    analogWrite(MOTOR_B1, 60);    // Speed for right motor
    failSafe();
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
  int rotations = round((distance / WheelC) * 20);

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= rotations && LRotation <= rotations) {
    back_Pixel();
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B2, LOW);
    analogWrite(MOTOR_A1, 255);
    analogWrite(MOTOR_B1, 255);
  }
  stop_();
}

// void backwards(int distance) {
//   stop_(); // Ensure the robot is stationary before starting

//   // Calculate the required number of wheel rotations based on distance
//   int rotations = round((distance / WheelC) * 20);

//   // Reset rotation counters
//   noInterrupts();
//   RRotation = 0;
//   LRotation = 0;
//   interrupts();

//   // Start backward movement
//   while (RRotation <= rotations || LRotation <= rotations) {
//     if (RRotation <= rotations) {
//       digitalWrite(MOTOR_A2, LOW);  // Right motor backward
//       analogWrite(MOTOR_A1, 255);  // Adjust speed as needed
//     } else {
//       digitalWrite(MOTOR_A2, LOW);
//       analogWrite(MOTOR_A1, 0);    // Stop right motor when done
//     }

//     if (LRotation <= rotations) {
//       digitalWrite(MOTOR_B2, LOW);  // Left motor backward
//       analogWrite(MOTOR_B1, 255);  // Adjust speed as needed
//     } else {
//       digitalWrite(MOTOR_B2, LOW);
//       analogWrite(MOTOR_B1, 0);    // Stop left motor when done
//     }

//     //failSafe(); // Ensure robot safety
//   }

//   // Stop the robot after completing the backward movement
//   stop_();
// }


void turn_r() {
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (LRotation < 40 && RRotation < 20) { // A is left B is right
  if (LRotation < 40) {
      digitalWrite(MOTOR_A2, HIGH);
      analogWrite(MOTOR_A1, 0);
    } else {
      digitalWrite(MOTOR_A2, LOW);
      analogWrite(MOTOR_A1, 0);
    }    
    if (RRotation < 20) {
      digitalWrite(MOTOR_B2, HIGH);
      analogWrite(MOTOR_B1, 180);
    } else {
      digitalWrite(MOTOR_B2, LOW);
      analogWrite(MOTOR_B1, 0);
    }
    failSafe();
  }
  stop_();
}

void turn_l() { // changed it to turn a very small amount jut for corrections
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (LRotation < 20 && RRotation < 40) { // A is left B is right
  if (LRotation < 20) {
      digitalWrite(MOTOR_A2, HIGH);
      analogWrite(MOTOR_A1, 100);
    } else {
      digitalWrite(MOTOR_A2, LOW);
      analogWrite(MOTOR_A1, 0);
    }    
    if (RRotation < 40) {
      digitalWrite(MOTOR_B2, HIGH);
      analogWrite(MOTOR_B1, 10);
    } else {
      digitalWrite(MOTOR_B2, LOW);
      analogWrite(MOTOR_B1, 0);
    }
  }
  stop_();
}

void rotate_r() { // should be done
  stop_();

  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();

  while (RRotation <= 13 && LRotation <= 13) {
    analogWrite(MOTOR_A1, 25);
    digitalWrite(MOTOR_B2, LOW);

    analogWrite(MOTOR_B1, 255);
    digitalWrite(MOTOR_A2, HIGH);
    failSafe();
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
    failSafe();
  }
  analogWrite(MOTOR_A1, 25);
  digitalWrite(MOTOR_B2, LOW);
  analogWrite(MOTOR_B1, 255);
  digitalWrite(MOTOR_A2, HIGH);
  delay(10);
  stop_();
}

void keepDistance() {
  const float errorRange = 5.0;
  const float targetDistance = 10.0;

  if (right_dis >= targetDistance - errorRange && right_dis <= targetDistance + errorRange) {
    forward(5);    
  } else if (right_dis >= targetDistance - errorRange) {
    turn_l();
  }  else if (right_dis >= targetDistance + errorRange) {
    turn_r();
  }
}

void failSafe() {
  static unsigned long lastRotationTime = millis(); // Record the last time rotations were updated
  static int lastRRotation = 0; // Track last rotation count for the right wheel
  static int lastLRotation = 0; // Track last rotation count for the left wheel

  // Check if rotations have changed
  if (RRotation != lastRRotation || LRotation != lastLRotation) {
    lastRotationTime = millis(); // Update the time if rotations changed
    lastRRotation = RRotation;   // Update the last known rotations
    lastLRotation = LRotation;
  } else if (millis() - lastRotationTime > 1000) { // If no change for 2 seconds
    Serial.println("FailSafe Triggered: No movement detected!");
    backwards(15); // Move backward as a fail-safe response
    lastRotationTime = millis(); // Reset the timer after moving backward
  }
}


// sonar
void sonar1() // forward sonar
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
      forward_dis = distance[1];
    }
    Serial.print("forward Distance: ");
    Serial.println(forward_dis);
}

void sonar2() // right sonar
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
      right_dis = distance[1];
    }
    // Serial.print("right Distance: ");
    // Serial.println(right_dis);
}

void gameLogic() {
  updateSensor1();
  updateSensor2();
  if(right_dis > 20)
  {
    Serial.println("rotate right and forward");
    turn_r();
    updateSensor1();
    updateSensor2();
    if(forward_dis > 15)
    {
      Serial.println("forward after right");
      forward(15);
      updateSensor1();
      updateSensor2();
    }
  }
  else if(forward_dis > 15)
  {
    Serial.println("forward");
    forward(15);
    updateSensor1();
    updateSensor2();
  }
  else if(forward_dis < 15 && right_dis < 15)
  {
    Serial.println("rotate LEFT");
    updateSensor1();
    updateSensor2();
    if(forward_dis < 13 && right_dis < 13)
    {
      backwards(20);
      updateSensor1();
      updateSensor2();
    }
    if(forward_dis > 15)
    {
      Serial.print("forward");
      forward(15);
      updateSensor1();
      updateSensor2();
    }
  }

  if(forward_dis < 15)
  {
    Serial.print("back");
    backwards(15);
    updateSensor1();
    updateSensor2();
  }
}


void logictest()
{
  updateSensor1();
  updateSensor2();
  while(forward_dis > 15)
  {
    updateSensor1();
    updateSensor2();
    if(right_dis >  20)
    {
      turn_r();
      updateSensor1();
      updateSensor2();
      if(forward_dis > 20)
      {
        forward(20);
        updateSensor1();
        updateSensor2();
      }
    }
    forward(15);
  }
  if(right_dis > 20)
  {
    turn_r();
    updateSensor1();
    updateSensor2();
    if(forward_dis > 30)
      {
        forward(20);
        updateSensor1();
        updateSensor2();
      }
  }
  if(right_dis < 15 && right_dis > 1 && forward_dis < 15 && forward_dis > 1)
  {
    rotate_l();
    delay(1500);
    updateSensor1();
    updateSensor2();
  }
  if(forward_dis < 15)
  {
    backwards(15);
    updateSensor1();
    updateSensor2();
  }
}

// int calculateRotationCount(int degrees) {
//     float wheelCircumference = PIVALUE * 6.5;  // Wheel diameter is 6.5 cm
//     float robotWidth = 15.0;  // Distance between wheels in cm
//     float rotationCircumference = PIVALUE * robotWidth;
//     float distanceToTravel = (degrees / 360.0) * rotationCircumference;
//     return round((distanceToTravel / wheelCircumference) * 40);  // 40 encoder ticks per wheel rotation
// }

// void preciseRotate_r(int degrees) {
//     int targetRotation = calculateRotationCount(degrees);
//     int slowdownThreshold = targetRotation * 0.8;
    
//     noInterrupts();
//     RRotation = 0;
//     LRotation = 0;
//     interrupts();
    
//     while (RRotation < targetRotation && LRotation < targetRotation) {
//         int speed = (RRotation < slowdownThreshold) ? 50 : 25;
        
//         digitalWrite(MOTOR_A2, HIGH);  // Right motor forward
//         digitalWrite(MOTOR_B2, LOW);   // Left motor backward
//         analogWrite(MOTOR_A1, speed);
//         analogWrite(MOTOR_B1, speed);
//     }
    
//     stop_();
// }
int PID(int steeringError)
{
   // Proportional term
  float proportional = Kp * steeringError;

  // Integral term
  integral += steeringError;  // Accumulate error over time
  float integralTerm = Ki * integral;

  // Derivative term
  float derivative = steeringError - prevError;  // Rate of change of error
  float derivativeTerm = Kd * derivative;

  // Update previous error for the next loop
  prevError = steeringError;

  // Return the combined PID output
  return proportional + integralTerm + derivativeTerm;
}

int steering_Error()
{
  updateSensor1();
  updateSensor2();
if(right_dis > 9 || right_dis < 11)
{
  steeringError = right_dis - 10;
}
else
{
  steeringError = 0;
}
  // Serial.println("steeringError: ");
  // Serial.println(steeringError);
  // Serial.println("right_dis: ");
  // Serial.println(right_dis);
return steeringError;
}

void adjustSteering(int steeringError) 
{
  //steeringError = steering_Error();
  // Calculate PID output based on the steering error
  float PID_output = PID(steeringError);

  // Base speed for motors when moving forward
  int baseSpeed = 200; // Adjust this as necessary for your robot
  steering_Error();
  failSafe();
  // Serial.print("Steering Error: ");
  // Serial.print(steeringError);
  // Serial.print(" | PID Output: ");
  // Serial.print(PID_output);
  if(forward_dis < 15 && steeringError < 6 && steeringError != 0)
  {
    rotate_l();
    delay(1000);
    steering_Error();
    if(forward_dis > 20)
    {
      forward(30);
      // steering_Error();
      return;
    }
  }
  if(steeringError >= 10)
  {
    right_Pixel();
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_A1, 20);
    analogWrite(MOTOR_B1, 160);
  }
  // Adjust movement based on the steering error
  if(steeringError < 2 && steeringError >= -1 && steeringError != 0)
  {
    // Robot is aligned, move straight
    normal_Pixel();
    digitalWrite(MOTOR_A2, HIGH); // Left motor forward
    analogWrite(MOTOR_A1, 60);      // Ensure left motor doesn't go backward
    analogWrite(MOTOR_B1, 60); // Right motor forward
    digitalWrite(MOTOR_B2, HIGH);      // Ensure right motor doesn't go backward
    // steering_Error();
  }
  else if (steeringError >= 2 && steeringError <= 5)
  {
    // Robot needs to turn right
    right_Pixel();
    int leftSpeed = baseSpeed + PID_output;  // Increase speed for left motor
    int rightSpeed = baseSpeed - PID_output; // Reduce speed for right motor

    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Apply motor speeds EDITED THIS PART****************************************
    digitalWrite(MOTOR_A2, HIGH); // Left motor forward
    analogWrite(MOTOR_A1, 60);      // Ensure left motor doesn't go backward
    analogWrite(MOTOR_B1, 85); // Right motor forward
    digitalWrite(MOTOR_B2, HIGH);       // Ensure right motor doesn't go backward
    // steering_Error();
    // Serial.print("leftspeed: ");
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
  }
  else if(steeringError > 5 && steeringError < 10)
  {
    right_Pixel();
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_A1, 60);
    analogWrite(MOTOR_B1, 95);
    // steering_Error();
  }
  else if (steeringError == 0)
  {
    // Apply motor speeds
    normal_Pixel();
    digitalWrite(MOTOR_A2, HIGH); 
    analogWrite(MOTOR_A1, 60);      
    analogWrite(MOTOR_B1, 60); 
    digitalWrite(MOTOR_B2, HIGH); 
    // steering_Error();      
  }
  else if (steeringError <= -2 && steeringError >= -4)
  {
    // Robot needs to turn left
    int leftSpeed = baseSpeed - PID_output;  // Reduce speed for left motor
    int rightSpeed = baseSpeed + PID_output; // Increase speed for right motor

    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Apply motor speeds
    left_Pixel();
    digitalWrite(MOTOR_A2, HIGH); // Left motor forward
    analogWrite(MOTOR_A1, 85);      // Ensure left motor doesn't go backward
    analogWrite(MOTOR_B1, 60); // Right motor forward
    digitalWrite(MOTOR_B2, HIGH);       // Ensure right motor doesn't go backward
    // steering_Error();
  }
  else if(steeringError < -4)
  {
    left_Pixel();
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B2, HIGH);
    analogWrite(MOTOR_A1, 95);
    analogWrite(MOTOR_B1, 60);
    // steering_Error();
  }

  // Optional: Debugging output
  // Serial.print("Steering Error: ");
  // Serial.print(steeringError);
  // Serial.print(" | PID Output: ");
  // Serial.print(PID_output);
  // Serial.print(" | Left Speed: ");
  // Serial.print((steeringError >= 0) ? baseSpeed + PID_output : baseSpeed - PID_output);
  // Serial.print(" | Right Speed: ");
  // Serial.println((steeringError >= 0) ? baseSpeed - PID_output : baseSpeed + PID_output);
}

// void adjustSteering(int steeringError) 
// {
//   // Calculate PID output based on the steering error
//   float PID_output = PID(steeringError);

//   // Determine motor speeds based on the PID output
//   int baseSpeed = 150;  // Base speed for motors
//   int leftMotorSpeed = baseSpeed - PID_output;
//   int rightMotorSpeed = baseSpeed + PID_output;

//   // Ensure motor speeds are within valid PWM range (0 to 255)
//   leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
//   rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

//   // Apply motor speeds to control steering
//   analogWrite(MOTOR_A1, 255 - leftMotorSpeed); // Adjust left motor speed
//   digitalWrite(MOTOR_A2, leftMotorSpeed > 0 ? HIGH : LOW);

//   analogWrite(MOTOR_B1, 255 - rightMotorSpeed); // Adjust right motor speed
//   digitalWrite(MOTOR_B2, rightMotorSpeed > 0 ? HIGH : LOW);

//   // Debugging output
//   Serial.print("PID Output: ");
//   Serial.println(PID_output);
//   Serial.print("Left Motor Speed: ");
//   Serial.println(leftMotorSpeed);
//   Serial.print("Right Motor Speed: ");
//   Serial.println(rightMotorSpeed);
// }
void flagCheck()
{
  if(flagUp == false)
  {
    Serial.println("Stuck before");
    // if (distance[0] > 0 && distance[1] > 0 && abs(distance[0] - distance[1]) < 20)
    if (forward_dis > 20)
      {
        Serial.println("Does it run");
        flagUp = true;
        Serial.println("flagUp = true");
      }
  }
}

void grabCone()
{
    if (!hasExecuted) // Check if it hasn't executed and flagUp is true
    {
        gripper(1800);  // gripper open
        forward(40); // move forward till cone
          for (int i = 0; i < 10; i++) {  // close gripper 10 pulses per 200 mx
            gripper(950);
            delay(20);
          }
        forward(10); // move forward until over square
        rotate_l();  // spin onto track
        forward(40);
        stop_();
        hasExecuted = true; // Mark as executed
    }
}
void forwardRotation1()
{
  reset_Rotations();

  // Start moving forward
  analogWrite(MOTOR_A1, 60);
  digitalWrite(MOTOR_A2, HIGH); 

  analogWrite(MOTOR_B1, 60);
  digitalWrite(MOTOR_B2, HIGH);

  // Wait until both motors have completed 20 rotations
  while (RRotation < 35 && LRotation < 35) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop_();
}

// move forward over square
void forwardRotation2()
{
  reset_Rotations();

  // Start moving forward
  analogWrite(MOTOR_A1, 60);
  digitalWrite(MOTOR_A2, HIGH); 

  analogWrite(MOTOR_B1, 255);
  digitalWrite(MOTOR_B2, LOW);

  // Wait until both motors have completed 20 rotations
  while (RRotation < 10 && LRotation < 10) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop_();
}

void spin_Left1() // spin left on its axle in a 90 degree angle
{
    reset_Rotations();

  // Start moving forward
  analogWrite(MOTOR_A1, 190);
  digitalWrite(MOTOR_A2, LOW); 

  analogWrite(MOTOR_B1, 0);
  digitalWrite(MOTOR_B2, HIGH);

  // Wait until both motors have completed 20 rotations
  while (RRotation < 20 && LRotation < 20) 
  {
    // Do nothing, just wait
    delay(10);  // Small delay to prevent CPU hogging
  }

  // Stop motors after reaching 20 rotations
  stop_();
}


void gripper (int pulse) // 1000 closed   2000 open angle
{
  // int pulseWidth = map(angle, 0, 180, 1000, 2000); // -10 closed 130 open
  static unsigned long timer;
  static int lastPulse;
  if (pulse == 0)
  {
    pulse = lastPulse;
  }
  else
  {
    lastPulse = pulse;
  }
    if (millis() > timer)
    {
      digitalWrite (GRIPPER, HIGH);
      delayMicroseconds(pulse);
      digitalWrite(GRIPPER, LOW);
      timer = millis() + SERVO_INTERVAL;
      // Serial.println(timer);
    }
}

void reset_Rotations()
{
  noInterrupts();
  RRotation = 0;
  LRotation = 0;
  interrupts();
}

void Initializing()
{
  for (int i = 0; i < 8; i++) {
    pinMode(linePins[i], INPUT);
    sensorStates[i] = "white"; // Initial state for all sensors

    // Initialize recent readings
    for (int j = 0; j < AVERAGE_WINDOW; j++) {
      recentReadings[i][j] = 0;
    }
  }
}

void updateLineCalibration() {
  static unsigned long timer = 0;
  if (millis() - timer > 1000) {
    lineCalibration();
  }
  timer = millis(); // update every 0.25s can change
}


void calibrateSensors() {
  // Serial.println("Calibrating sensors...");

  for (int i = 0; i < 10; i++) {  // Adjust the number of calibration iterations as needed
    for (int j = 0; j < 8; j++) {
      int sensorValue = analogRead(linePins[j]);

      // Update minimum and maximum values
      if (sensorValue < minlineValue[j]) {
        minlineValue[j] = sensorValue;
      }
      if (sensorValue > maxlineValue[j]) {
        maxlineValue[j] = sensorValue;
      }
      sensorValues[j] = sensorValue;
    }
  }
   // Calculate static thresholds for each sensor
  for (int i = 0; i < 8; i++) {
  High_Threshold[i] = 800; //(maxlineValue[i] + minlineValue[i]) / 2 + 100; // Black threshold
  Low_Threshold[i] =  High_Threshold[i] - 50;//( maxlineValue[i] + minlineValue[i]) / 2 - 100; // Adjusted white threshold (higher range)
  }
  // Serial.println("Calibration complete!");
}

void lineCalibration() {
  for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(linePins[i]);

    // Update recent readings
    recentReadings[i][readingIndex] = sensorValue;

    // Calculate moving average
    long sum = 0;
    for (int j = 0; j < AVERAGE_WINDOW; j++) {
      sum += recentReadings[i][j];
    }
    avglineValue[i] = sum;
  }

  // Update reading index
  readingIndex = (readingIndex + 1) % AVERAGE_WINDOW;

  //delay(200); // Delay for stability
}

void determineStates() {
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] >= High_Threshold[i]) {
      sensorStates[i] = 1;// black
    } else if (sensorValues[i] <= Low_Threshold[i]) {
      sensorStates[i] = 0;// white
    } else {
      // In the gap between thresholds, maintain the previous state
      sensorStates[i] = previousSensorStates[i];
    }

    // Update the previous state after determining the current state
    previousSensorStates[i] = sensorStates[i];
  }
}

void line_steerError()
{
   // Variable to store the calculated error for steering
    line_steeringError = 0; // Reset error
    int totalWeight = 0;    // Track the number of active (black) sensors
   // Read all sensor values and calculate the weighted sum (steering error)
   for (int i =0; i < 8; i++)
   {
    if (sensorStates[i] == 1)// check for black
    {
      line_steeringError += sensorWeights[i];  // Add weighted sensor value to the total error
      totalWeight++;
    }
   }
   // Normalize error if any black sensors are active
    if (totalWeight > 0) 
    {
        line_steeringError /= totalWeight;
    }
    // Debug output
    Serial.print("Steering Error: ");
    Serial.println(line_steeringError);
}

void dropCone()
{
  // updateLineCalibration();
  // calibrateSensors();
  // determineStates();
  //line_steerError();
  if(line_steeringError == 2 )
  {
    stop_();
    for(int i = 0; i <10; i++ )
    {
      gripper(2000);
    }
    backwards(20);
    stop_();
    delay(10000);
  }
}

// void line_adjustSteering(int line_steeringError) 
// {
//   // Calculate PID output based on the steering error
//   updateLineCalibration();
//   calibrateSensors();
//   determineStates();
//   // line_steerError();
//   Serial.print("line_steeringError: ");
//   Serial.println(line_steeringError);
//   // while (line_steeringError == 0)
//   // {
//   //   // Apply motor speeds
//   //   steering_Error();
//   //   adjustSteering(steeringError);
//   //   Serial.println("line_steeringError: ");
//   //   Serial.print(line_steeringError);      
//   // }
//   float PID_output = PID(line_steeringError);

//   // Base speed for motors when moving forward
//   int baseSpeed = 200; // Adjust this as necessary for your robot

//   // Adjust movement based on the steering error
//   if (line_steeringError <= 4 && line_steeringError >= -4 && line_steeringError != 0 && line_steeringError != 2)
//   {
//     // Robot is aligned, move straight
//     digitalWrite(MOTOR_A2, HIGH); 
//     analogWrite(MOTOR_A1, 60);      
//     analogWrite(MOTOR_B1, 60); 
//     digitalWrite(MOTOR_B2, HIGH); 
//   }
//   else if (line_steeringError > 6)
//   {
//     // Robot needs to turn right
//     int leftSpeed = baseSpeed + PID_output;  // Increase speed for left motor
//     int rightSpeed = baseSpeed - PID_output; // Reduce speed for right motor

//     // Constrain speeds to valid PWM range
//     leftSpeed = constrain(leftSpeed, 0, 255);
//     rightSpeed = constrain(rightSpeed, 0, 255);

//     // Apply motor speeds
//     digitalWrite(MOTOR_A2, HIGH);
//     digitalWrite(MOTOR_B2, HIGH);
//     analogWrite(MOTOR_A1, 60);
//     analogWrite(MOTOR_B1, 95);
//   }
//   else if (line_steeringError < -6)
//   {
//     // Robot needs to turn left
//     int leftSpeed = baseSpeed - PID_output;  // Reduce speed for left motor
//     int rightSpeed = baseSpeed + PID_output; // Increase speed for right motor

//     // Constrain speeds to valid PWM range
//     leftSpeed = constrain(leftSpeed, 0, 255);
//     rightSpeed = constrain(rightSpeed, 0, 255);

//     // Apply motor speeds
//     digitalWrite(MOTOR_A2, HIGH);
//     digitalWrite(MOTOR_B2, HIGH);
//     analogWrite(MOTOR_A1, 95);
//     analogWrite(MOTOR_B1, 60);      // Ensure right motor doesn't go backward
//   }

//   if(line_steeringError == 2 )
//   {
//     stop_();
//     for(int i = 0; i <10; i++ )
//     {
//       gripper(2000);
//     }
//     backwards(20);
//     stop_();
//     delay(10000);
//   }
  

//   // Optional: Debugging output
//   // Serial.print("Steering Error: ");
//   // Serial.print(steeringError);
//   // Serial.print(" | PID Output: ");
//   // Serial.print(PID_output);
//   // Serial.print(" | Left Speed: ");
//   // Serial.print((steeringError >= 0) ? baseSpeed + PID_output : baseSpeed - PID_output);
//   // Serial.print(" | Right Speed: ");
//   // Serial.println((steeringError >= 0) ? baseSpeed - PID_output : baseSpeed + PID_output);
// }

void line_adjustSteering(int line_steeringError) 
{
  updateLineCalibration();
  calibrateSensors();
  determineStates();
    if (line_steeringError == 0)
  {
    // Apply motor speeds
    steering_Error();
    adjustSteering(steeringError);
    // Serial.print("line_steeringError: ");
    // Serial.println(line_steeringError);      
  }
  // Calculate PID output based on the steering error
  float PID_output = PID(line_steeringError);

  // Base speed for motors when moving forward
  int baseSpeed = 255; // Adjust this as necessary for your robot 235

    // Update last known steering error if line is detected
  if (line_steeringError != 0) {
    lastKnownline_SteeringError = line_steeringError;
  }

  // Adjust movement based on the steering error
  if (line_steeringError <= 4 && line_steeringError >= -4 && line_steeringError != 0) // 7, -7 
  {
    // Robot is aligned, move straight
    digitalWrite(MOTOR_A2, HIGH); 
    analogWrite(MOTOR_A1, 60);      
    analogWrite(MOTOR_B1, 60); 
    digitalWrite(MOTOR_B2, HIGH);
  }
  else if (line_steeringError <= -6) // -11
  {
    // Robot needs to turn left
    int leftSpeed = baseSpeed + PID_output;  // Reduce speed for left motor
    int rightSpeed = baseSpeed - PID_output; // Increase speed for right motor

    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Apply motor speeds
    analogWrite(MOTOR_A1, 130); // Left motor forward
    digitalWrite(MOTOR_A2, HIGH);      // Ensure left motor doesn't go backward
    analogWrite(MOTOR_B1, 50); // Right motor forward
    digitalWrite(MOTOR_B2, HIGH);       // Ensure right motor doesn't go backward
  }
  else if (line_steeringError >= 6) // 11
  {
    // Robot needs to turn right
    int leftSpeed = baseSpeed - PID_output;  // Increase speed for left motor
    int rightSpeed = baseSpeed + PID_output; // Reduce speed for right motor

    // Constrain speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Apply motor speeds
    analogWrite(MOTOR_A1, 50); // Left motor forward
    digitalWrite(MOTOR_A2, HIGH);      // Ensure left motor doesn't go backward
    analogWrite(MOTOR_B1, 130); // Right motor forward
    digitalWrite(MOTOR_B2, HIGH);       // Ensure right motor doesn't go backward

  }
  // Optional: Debugging output
  // Serial.print("Steering Error: ");
  // Serial.print(line_steeringError);
  // Serial.print(" | PID Output: ");
  // Serial.print(PID_output);
    if(sensorStates[0] == 1 && sensorStates[1] == 1 && sensorStates[2] == 1 && sensorStates[3] == 1 && sensorStates[4] == 1 && sensorStates[5] == 1 && sensorStates[6] == 1 && sensorStates[7] == 1 )
  {
    stop_();
    for(int i = 0; i <10; i++ )
    {
      gripper(2000);
    }
    backwards(30);
    stop_();
    delay(10000);
  }
}

void normal_Pixel()
{
  //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(75,0,0)); // red at 60 left back
  pixels.setPixelColor(1, pixels.Color(75,0,0)); // red at 60 right back
  pixels.setPixelColor(2, pixels.Color(255, 255, 75)); // white ish headlights 100 right front
  pixels.setPixelColor(3, pixels.Color(255, 255, 75)); // white ish headlights 100 left front
  pixels.show();
}

void braking_Pixel()
{
  //pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255,0,0)); // red at 100 left back
  pixels.setPixelColor(1, pixels.Color(255,0,0)); // red at 100 right back
  pixels.setPixelColor(2, pixels.Color(255, 255, 75)); // white ish headlights 100 right front
  pixels.setPixelColor(3, pixels.Color(255, 255, 75)); // white ish headlights 100 left front
  pixels.show();
}

  void right_Pixel()
  {
    static bool lightOn = false;
    static unsigned long timer = millis();

    if ((millis() - timer) > 500)
    {
      if(lightOn == true)
      {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(75, 0, 0));      // Red at 60 left back
        pixels.setPixelColor(1, pixels.Color(255, 100, 0));   // Orange right
        pixels.setPixelColor(2, pixels.Color(255, 100, 0));   // Orange right
        pixels.setPixelColor(3, pixels.Color(255, 255, 75));  // White-ish headlights left front
        pixels.show();
      }
      else
      {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(75, 0, 0));      // Red at 60 left back
        pixels.setPixelColor(1, pixels.Color(0,0,0));         // off right
        pixels.setPixelColor(2, pixels.Color(0,0,0));         // off right
        pixels.setPixelColor(3, pixels.Color(255, 255, 75));  // White-ish headlights left front
        pixels.show(); // Turn off the orange pixels 
      }
      lightOn = !lightOn;
      timer = millis();
    }
  }


  void left_Pixel()
  {
    static bool lightOn = false;
    static unsigned long timer = millis();

    if ((millis() - timer) > 500)
    {
      if(lightOn == true)
      {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(255, 100, 0));   // Orange left back
        pixels.setPixelColor(1, pixels.Color(75, 0, 0));      // Red at 60 right back
        pixels.setPixelColor(2, pixels.Color(255, 255, 75));  // White-ish headlights right front
        pixels.setPixelColor(3, pixels.Color(255, 100, 0));   // Orange left front
        pixels.show();
      }
      else
      {
        pixels.clear();
        pixels.setPixelColor(1, pixels.Color(75, 0, 0));      // Red at 60 right back
        pixels.setPixelColor(0, pixels.Color(0,0,0));         // off left back
        pixels.setPixelColor(3, pixels.Color(0,0,0));         // off left front
        pixels.setPixelColor(2, pixels.Color(255, 255, 75));  // White-ish headlights right front
        pixels.show(); // Turn off the orange pixels
      }
      lightOn = !lightOn;
      timer = millis();
    }
  }


void back_Pixel()
{
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(255,0,0)); // red at 100 left back
  pixels.setPixelColor(1, pixels.Color(255,255,255)); // red at 100 right back
  pixels.setPixelColor(2, pixels.Color(255, 255, 75)); // white ish headlights 100 right front
  pixels.setPixelColor(3, pixels.Color(255, 255, 75)); // white ish headlights 100 left front
  pixels.show();
}
