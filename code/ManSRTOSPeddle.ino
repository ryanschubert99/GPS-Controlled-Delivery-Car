#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include "esp_system.h"

BluetoothSerial SerialBT;

// Diagnostic LED (on-board usually GPIO 2)
const int diagLedPin = 2;

// Motor A pins
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 14;

// Servo setup
Servo myServo;
const int servoPin    = 12;
int servoAngle        = 90;
const int tickStep    = 13;

// PWM properties
const int freq        = 60000;
const int pwmChannel  = 0;
const int resolution  = 10;

// Speed variables
int currentSpeedPercentage = 0;
bool goingForward          = true;
const int maxSpeed         = 1024;

// Queues
QueueHandle_t motorQueue;
QueueHandle_t servoQueue;

// Diagnostics state
bool wasConnected    = false;
unsigned long lastHeartbeat = 0;

// Convert reset reason to string
const char* resetReasonToString(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:    return "Unknown";
    case ESP_RST_POWERON:    return "Power-on";
    case ESP_RST_EXT:        return "External reset";
    case ESP_RST_SW:         return "Software reset";
    case ESP_RST_PANIC:      return "Panic";
    case ESP_RST_INT_WDT:    return "Interrupt WDT";
    case ESP_RST_TASK_WDT:   return "Task WDT";
    case ESP_RST_WDT:        return "Other WDT";
    case ESP_RST_DEEPSLEEP:  return "Deep Sleep";
    case ESP_RST_BROWNOUT:   return "Brownout";
    case ESP_RST_SDIO:       return "SDIO";
    default:                 return "??";
  }
}

// Servo turning
void turnServo(const String &direction) {
  if (direction == "rr") {
    if (servoAngle < 180) {
      servoAngle += tickStep;
      myServo.write(servoAngle);
    }
    Serial.print("Turning right. Servo angle: ");
    Serial.println(servoAngle);
  } else if (direction == "ll") {
    if (servoAngle > 0) {
      servoAngle -= tickStep;
      myServo.write(servoAngle);
    }
    Serial.print("Turning left. Servo angle: ");
    Serial.println(servoAngle);
  } else {
    Serial.println("Invalid input for servo.");
  }
}

// Motor movement
void moveMotor(int speedPercentage, const String &direction) {
  int dutyCycle = (speedPercentage * maxSpeed) / 100;

  if (direction == "forward") {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }

  ledcWrite(enable1Pin, dutyCycle);

  Serial.print("Moving ");
  Serial.print(direction);
  Serial.print(" at ");
  Serial.print(speedPercentage);
  Serial.print("% (");
  Serial.print(dutyCycle);
  Serial.println(" duty cycle)");
}

// Motor control task
void motorControlTask(void *parameter) {
  String motorCommand;
  unsigned long lastPedalTime = 0;
  const int decelInterval = 150;
  const int noPressTimeout = 400;

  while (true) {
    if (!SerialBT.hasClient()) {
      ledcWrite(enable1Pin, 0);
      currentSpeedPercentage = 0;
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    if (xQueueReceive(motorQueue, &motorCommand, 10 / portTICK_PERIOD_MS)) {
      if (motorCommand == "ff") {
        goingForward = true;
        lastPedalTime = millis();
        currentSpeedPercentage = min(currentSpeedPercentage + 5, 100);
        moveMotor(currentSpeedPercentage, "forward");
      } 
      else if (motorCommand == "bb") {
        goingForward = false;
        lastPedalTime = millis();
        currentSpeedPercentage = min(currentSpeedPercentage + 5, 100);
        moveMotor(currentSpeedPercentage, "backward");
      } 
      else if (motorCommand == "ss") {
        lastPedalTime = 0;
      }
    }

    if (millis() - lastPedalTime > noPressTimeout && currentSpeedPercentage > 0) {
      static unsigned long lastDecel = 0;
      if (millis() - lastDecel >= decelInterval) {
        currentSpeedPercentage = max(currentSpeedPercentage - 30, 0);
        moveMotor(currentSpeedPercentage, goingForward ? "forward" : "backward");
        lastDecel = millis();
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Servo control task
void servoControlTask(void *parameter) {
  String servoCommand;
  while (true) {
    if (xQueueReceive(servoQueue, &servoCommand, portMAX_DELAY)) {
      turnServo(servoCommand);
    }
  }
}

void setup() {
  // Diagnostic LED
  pinMode(diagLedPin, OUTPUT);
  digitalWrite(diagLedPin, LOW);

  // Motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);

  // Servo
  myServo.attach(servoPin);
  myServo.write(servoAngle);

  // Serial
  Serial.begin(115200);
  delay(500);

  // Print reset reason
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("⟳ Boot! Reset reason: ");
  Serial.println(resetReasonToString(reason));

  // Start Bluetooth SPP (no PIN)
  if (!SerialBT.begin("ESP32test")) {
    Serial.println("❌ BT init failed!");
    while (1) { delay(100); }
  }
  Serial.println("✅ Bluetooth SPP started as \"ESP32test\"");

  // Create queues and tasks
  motorQueue = xQueueCreate(10, sizeof(String));
  servoQueue = xQueueCreate(10, sizeof(String));
  xTaskCreatePinnedToCore(motorControlTask, "Motor Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(servoControlTask, "Servo Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
  // Connection diagnostics
  bool isConnected = SerialBT.hasClient();
  if (isConnected != wasConnected) {
    Serial.println(isConnected ? "🔗 Client connected" : "❌ Client disconnected");
    digitalWrite(diagLedPin, isConnected ? HIGH : LOW);
    wasConnected = isConnected;
  }

  // Heartbeat
  if (millis() - lastHeartbeat >= 5000) {
    lastHeartbeat = millis();
    Serial.print("⏱ Uptime: ");
    Serial.print(millis() / 1000);
    Serial.print(" s, Free heap: ");
    Serial.println(ESP.getFreeHeap());
  }

  // Handle incoming BLE commands
  if (SerialBT.available()) {
    String input;
    while (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '\n' || c == '\r') break;
      input += c;
    }
    input.trim();
    input.toLowerCase();

    // Normalize single-letter to double-letter
    if      (input == "f") input = "ff";
    else if (input == "b") input = "bb";
    else if (input == "s") input = "ss";
    else if (input == "r") input = "rr";
    else if (input == "l") input = "ll";
    else if (input == "i") input = "ii";
    else if (input == "g") input = "gg";
    else if (input == "h") input = "hh";
    else if (input == "j") input = "jj";

    Serial.print("Received input: ");
    Serial.println(input);

    // Dispatch commands
    if      (input == "ii") {
      String m = "ff", s = "rr";
      xQueueSend(motorQueue, &m, portMAX_DELAY);
      xQueueSend(servoQueue, &s, portMAX_DELAY);
    }
    else if (input == "gg") {
      String m = "ff", s = "ll";
      xQueueSend(motorQueue, &m, portMAX_DELAY);
      xQueueSend(servoQueue, &s, portMAX_DELAY);
    }
    else if (input == "hh") {
      String m = "bb", s = "ll";
      xQueueSend(motorQueue, &m, portMAX_DELAY);
      xQueueSend(servoQueue, &s, portMAX_DELAY);
    }
    else if (input == "jj") {
      String m = "bb", s = "rr";
      xQueueSend(motorQueue, &m, portMAX_DELAY);
      xQueueSend(servoQueue, &s, portMAX_DELAY);
    }
    else if (input == "ff" || input == "bb" || input == "ss") {
      xQueueSend(motorQueue, &input, portMAX_DELAY);
    }
    else if (input == "rr" || input == "ll") {
      xQueueSend(servoQueue, &input, portMAX_DELAY);
    }
  }

  delay(10);
}
