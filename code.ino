// Arduino Line Follower with PID, Sharp Turns & HC-06 Bluetooth Control
// Board: Arduino Uno R3

#include <SoftwareSerial.h>

// Bluetooth pins
#define BT_RX 2
#define BT_TX 3
SoftwareSerial BT(BT_RX, BT_TX); // RX, TX

// IR sensor pins
#define IR_L2 A0
#define IR_L1 A1
#define IR_M  A2
#define IR_R1 A3
#define IR_R2 A4

// Motor control pins
#define ENA 5   // PWM left
#define IN1 6   // left dir
#define IN2 7
#define ENB 9   // PWM right
#define IN3 10  // right dir
#define IN4 11

// PID constants
float Kp = 20.0;
float Ki = 0.0; 
float Kd = 30.0;

int baseSpeed = 120;
float error = 0, previousError = 0, integral = 0, derivative = 0;

int sensorValues[5];
int sensorWeights[5] = { -7, -3, 0, 3, 7 };
int bluetoothValues[5];
// Cornering and boost parameters
const int sharpTurnThreshold = 70;
const int turnBoost = 90;
const int recoverySpeed = 180;

// Sampling
unsigned long lastTime = 0;
const int sampleTime = 15;

// Bluetooth control
char btCommand = '\0';
bool manualMode = false;

void setup() {
  // Sensors
  pinMode(IR_L2, INPUT);
  pinMode(IR_L1, INPUT);
  pinMode(IR_M , INPUT);
  pinMode(IR_R1, INPUT);
  pinMode(IR_R2, INPUT);
  // Motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // init serial
  BT.begin(9600);
  Serial.begin(9600);
  // default motor dir forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void loop() {
  checkBluetooth();
  if (millis() - lastTime >= sampleTime) {
    lastTime = millis();
    readSensors();
    computePID();
    adjustMotors();
  }
  
}

void checkBluetooth() {
  if (BT.available()) {
    char temp = BT.read();
    Serial.write(btCommand);
    switch (temp) {
      case 'L':
        bluetoothValues[0]++;
        break; 
      case 'R': 
        bluetoothValues[1]++;
        break; 
      case 'M': 
        bluetoothValues[2]++;
        break; 
      case 'S':
        bluetoothValues[3]++;
        break; 
      
    }
  }
}

void determineBestCommand() {
  int maxIndex = 0;
  for (int i = 1; i < 4; i++) {
    if (bluetoothValues[i] > bluetoothValues[maxIndex]) {
      maxIndex = i;
    }
  }
  if (bluetoothValues[maxIndex] == 0) return;
  switch (maxIndex) {
    case 0: btCommand = 'L'; break;
    case 1: btCommand = 'R'; break;
    case 2: btCommand = 'M'; break;
    case 3: btCommand = 'S'; break;
  }
  for (int i = 0; i < 4; i++) {
    bluetoothValues[i] = 0;
  }
}

void executeManualCommand() {
  determineBestCommand();
  switch (btCommand) {
    case 'L': // sharp left
      driveMotors(-recoverySpeed - 40, recoverySpeed + 40);
      delay(600);
      break;
    case 'R': // sharp right
      driveMotors(recoverySpeed + 40, -recoverySpeed - 40);
      delay(600);
      break;
    case 'M': // move straight
      driveMotors(baseSpeed, baseSpeed);
      delay(700);
      break;
    case 'S': // stop
      driveMotors(0, 0);
      delay(1000000);
      break;
  }
  Serial.print(btCommand);
  btCommand = '\0';
  //manualMode = false;
  
}

void readSensors() {
  sensorValues[0] = !digitalRead(IR_L2);
  sensorValues[1] = !digitalRead(IR_L1);
  sensorValues[2] = !digitalRead(IR_M);
  sensorValues[3] = !digitalRead(IR_R1);
  sensorValues[4] = !digitalRead(IR_R2);
}

void computePID() {
  long weightedSum = 0;
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    weightedSum += (long)sensorValues[i] * sensorWeights[i];
    sum += sensorValues[i];
  }
  if (sum == 5) {
    Serial.println("Excute:");
    executeManualCommand();
    error = (previousError > 0) ? 2.0 : -2.0;
  } else if (sum > 0) {
    error = (float)weightedSum / sum;
  } else {
    error = (previousError < 0) ? -6.0 : 6.0;
  }
  if (abs(error) < 2.5 && sum > 0) {
    integral += error;
    integral = constrain(integral, -150, 150);
  } else {
    integral = 0;
  }
  derivative = error - previousError;
  previousError = error;
}

void adjustMotors() {
  float PIDoutput = Kp * error + Ki * integral + Kd * derivative;
  int leftSpeed  = constrain(baseSpeed + (int)PIDoutput, 0, 255);
  int rightSpeed = constrain(baseSpeed - (int)PIDoutput, 0, 255);

  if (PIDoutput > sharpTurnThreshold) {
    driveMotors(recoverySpeed, -recoverySpeed);
  } else if (PIDoutput < -sharpTurnThreshold) {
    driveMotors(-recoverySpeed, recoverySpeed);
  } else if (abs(PIDoutput) > 40) {
    if (PIDoutput > 0) leftSpeed = min(leftSpeed + turnBoost, 255);
    else              rightSpeed = min(rightSpeed + turnBoost, 255);
    driveMotors(leftSpeed, rightSpeed);
  } else {
    driveMotors(leftSpeed, rightSpeed);
  }
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // LEFT MOTOR
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, abs(leftSpeed));

  // RIGHT MOTOR
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, abs(rightSpeed));
}

