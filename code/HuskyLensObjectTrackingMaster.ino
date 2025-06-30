#include <HUSKYLENS.h>
#include <Wire.h>
#include <BluetoothSerial.h>

HUSKYLENS huskylens;
BluetoothSerial SerialBT;

// Ultrasonic sensor pins
const int trigPin = 18;
const int echoPin = 19;

// I2C address of the motor/servo ESP32 slave
const uint8_t SLAVE_ADDR = 0x08;

// Command bytes for the slave ESP32
#define CMD_TRACK_FORWARD 3
#define CMD_TURN_RIGHT    4
#define CMD_TURN_LEFT     5
#define CMD_STOP          6

// HuskyLens frame geometry
const int FRAME_WIDTH  = 320;
const int CENTER_X     = FRAME_WIDTH / 2;
const int TOLERANCE    = 40;  // ±px around center
const int DIST_THRESHOLD = 60; // cm

long duration;

// Measure distance via ultrasonic sensor
int getUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  //return duration * 0.034 / 2;
  return 70;
}

// Send a single-byte command over I2C to the slave
void sendCommand(uint8_t cmd) {
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
  Serial.printf("Sent command %d\n", cmd);
  SerialBT.printf("Sent command %d\n", cmd);
}

void setup() {
  // Serial + BLE for debug
  Serial.begin(115200);
  while (!Serial);
  SerialBT.begin("HuskyMaster");
  Serial.println("Serial & BLE started");

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // I2C for HuskyLens + slave
  Wire.begin();  // SDA=21, SCL=22

  // Initialize HuskyLens
  Serial.println("Connecting to HuskyLens...");
  while (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens begin failed! Check wiring and protocol.");
    delay(500);
  }
  Serial.println("HuskyLens connected.");

  // Switch to object-tracking algorithm
  //huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
  Serial.println("HuskyLens set to OBJECT_TRACKING mode.");
}

void loop() {
  // Request new data
  if (!huskylens.request()) {
    Serial.println("HuskyLens request failed.");
    sendCommand(CMD_STOP);
    delay(200);
    return;
  }

  // If an object is learned and available
  if (huskylens.isLearned() && huskylens.available()) {
    HUSKYLENSResult obj = huskylens.read();
    int x = obj.xCenter;
    int distance = getUltrasonicDistance();
    
    Serial.printf("Object at x=%d, dist=%dcm\n", x, distance);
    SerialBT.printf("Object at x=%d, dist=%dcm\n", x, distance);

    if (distance > DIST_THRESHOLD) {
      // Object far enough: decide turn vs forward
      if (abs(x - CENTER_X) <= TOLERANCE) {
        sendCommand(CMD_TRACK_FORWARD);
      }
      else if (x > CENTER_X) {
        sendCommand(CMD_TURN_RIGHT);
      }
      else {
        sendCommand(CMD_TURN_LEFT);
      }
    } else {
      // Too close: stop
      sendCommand(CMD_STOP);
    }
  } else {
    // No object on-screen: stop
    sendCommand(CMD_STOP);
  }

  delay(200);
}
