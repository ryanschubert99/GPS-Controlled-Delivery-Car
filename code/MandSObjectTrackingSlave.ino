#include <Wire.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include "esp_system.h"

BluetoothSerial SerialBT;
Servo myServo;

// Diagnostic LED (on-board usually GPIO 2)
const int diagLedPin = 2;

// I2C slave address
#define I2C_SLAVE_ADDR 0x08

// Received I2C command
volatile uint8_t i2cCommand = 0;
volatile bool i2cNew = false;

// Motor pins
const int motorPin1 = 27;
const int motorPin2 = 26;
const int enablePin = 14;

// Servo parameters
const int servoPin      = 12;
int curAngle            = 90;
const int straightAngle = 90;
const int turnAngle     = 40;  // degrees to turn for left/right

// Tracking speed
const int trackSpeed    = 20;  // % of max

// PWM properties
const int freq       = 60000;
const int pwmChannel = 0;
const int resolution = 10;

// ===== I2C receive event =====
void IRAM_ATTR onI2CReceive(int numBytes) {
  while (Wire.available()) {
    i2cCommand = Wire.read();
    i2cNew = true;
  }
}

// ===== Drive & Steer =====
void drive(int speedPct, bool forward) {
  int duty = (speedPct * ((1 << resolution) - 1)) / 100;
  digitalWrite(motorPin1, forward ? HIGH : LOW);
  digitalWrite(motorPin2, forward ? LOW  : HIGH);
  ledcWrite(enablePin, duty);
}

void stopDrive() {
  // immediate brake
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  ledcWrite(enablePin, 0);
}

int steerTo(int angle,int curAngle) {
  myServo.write(angle);
  // 3) wait just long enough for the horn to move
  int delta = abs(angle - curAngle);
  //float msPerDeg = 3.9;
  float msPerDeg = 3.5;
  delay(delta * msPerDeg + 10);

  curAngle = angle;
  return curAngle;
}

// ===== Tracking routines =====
void trackForward() {
  stopDrive();
  digitalWrite(diagLedPin, HIGH);
  steerTo(straightAngle,90);
  drive(trackSpeed, true);
  delay(100);
}

void turnRight() {
  stopDrive();
  digitalWrite(diagLedPin, HIGH);
  steerTo(straightAngle + turnAngle,90);
  drive(trackSpeed, true);
  delay(100);
}

void turnLeft() {
  stopDrive();
  digitalWrite(diagLedPin, HIGH);
  steerTo(straightAngle - turnAngle,90);
  drive(trackSpeed, true);
  delay(100);
}

void stopMovement() {
  digitalWrite(diagLedPin, LOW);
  stopDrive();
}

// ===== Setup =====
void setup() {
  // Diagnostic LED
  pinMode(diagLedPin, OUTPUT);
  digitalWrite(diagLedPin, LOW);

  // Motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  ledcAttachChannel(enablePin, freq, resolution, pwmChannel);

  // Servo
  myServo.attach(servoPin);
  myServo.write(straightAngle);
  curAngle = straightAngle;

  // I2C slave init
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onI2CReceive);

  // Serial & BLE
  Serial.begin(115200);
  SerialBT.begin("ESP32TrackSlave");
  SerialBT.println("BLE & I2C Tracking Slave Ready");
}

// ===== Main loop =====
void loop() {
  if (i2cNew) {
    uint8_t cmd = i2cCommand;
    i2cNew = false;
    SerialBT.printf("[I2C] Cmd=%d\n", cmd);

    switch (cmd) {
      case 3: trackForward();  break;  // forward
      case 4: turnRight();     break;
      case 5: turnLeft();      break;
      case 6: stopMovement();  break;
      default:
        SerialBT.println("[I2C] Unknown tracking cmd");
        stopMovement();
        break;
    }
  }
}
