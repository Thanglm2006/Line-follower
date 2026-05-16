#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"


// 8 line sensors connected from D26 up to EN on the ESP32 (left side of typical dev kit)
// The pins in order are: 26, 25, 33, 32, 35, 34, 39 (VN), 36 (VP)
const int numSensors = 8;
const int sensorPins[numSensors] = {26, 25, 33, 32, 35, 34, 39, 36};

// values correspond to the black line (lower reflection)
const int minValues[numSensors] = {2943, 3051, 2782, 2465, 2882, 2839, 3208, 2523};

// --- MOTOR PINS (TB6612FNG) ---
// Motor A (Right Motor)
const int PWMA = 27;
const int AIN1 = 18;
const int AIN2 = 19;

// Motor B (Left Motor)
const int PWMB = 13;
const int BIN1 = 5;
const int BIN2 = 23;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
const int OBSTACLE_THRESHOLD = 40; // 4cm = 40mm


// --- PID PARAMETERS ---
float Kp = 0.15; // Increased for high speed
float Kd = 1.0;  // Increased to dampen oscillations
float Ki = 0.0;

int baseSpeed = 100 ; // Speed during curves
int lastError = 0;


void setupMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Cho Arduino Core 3.0+
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
#else
  // Cho Arduino Core 2.x
  ledcSetup(0, 5000, 8); 
  ledcAttachPin(PWMA, 0);
  ledcSetup(1, 5000, 8); 
  ledcAttachPin(PWMB, 1);
#endif
}

// Right Motor (Motor A)
void setMotorRight(int speed) {
  if (speed >= 0) {
    // Forward (reversed)
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    // Backward (reversed)
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    speed = -speed;
  }
  
  if (speed > 255) speed = 255;
  
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWrite(PWMA, speed);
#else
  ledcWrite(0, speed);
#endif
}

// Left Motor (Motor B)
void setMotorLeft(int speed) {
  if (speed >= 0) {
    // Forward (reversed)
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    // Backward (reversed)
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    speed = -speed;
  }
  
  if (speed > 255) speed = 255;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWrite(PWMB, speed);
#else
  ledcWrite(1, speed);
#endif
}

// Function to turn around 180 degrees when obstacle is detected
void turnAround() {
  Serial.println("OBSTACLE DETECTED! Turning around...");
  
  // Stop motors
  setMotorLeft(0);
  setMotorRight(0);
  delay(300);
  
  // Spin in place
  setMotorLeft(-180);
  setMotorRight(180);
  
  // Small delay to get off the current line position
  delay(500); 
  
  // Continue turning until middle sensors detect the line
  bool lineFound = false;
  unsigned long startTurn = millis();
  while (!lineFound) {
    // Timeout to prevent infinite spinning if line is lost (3 seconds)
    if (millis() - startTurn > 3000) break;

    int val3 = analogRead(sensorPins[3]);
    int val4 = analogRead(sensorPins[4]);
    
    // Check if middle sensors detect black line
    if (val3 <= (minValues[3] + 400) || val4 <= (minValues[4] + 400)) {
      lineFound = true;
    }
    delay(5);
  }
  
  // Stop and stabilize
  setMotorLeft(0);
  setMotorRight(0);
  delay(1000);
  
  lastError = 0; // Reset PID error
}


void setup() {
  Serial.begin(115200);
  Serial.println("=== KHOI DONG ROBOT DO LINE ===");
  
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  setupMotors();
  
  // Initialize VL53L0X
  Wire.begin(21, 22);
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X sensor!");
  } else {
    Serial.println("VL53L0X OK");
  }
  
  // Wait 2 seconds before start moving
  delay(2000); 
}


void loop() {
  /* --- OBSTACLE SENSOR (DISABLED FOR SPEED) ---
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus <= 4) {
    int distance = measure.RangeMilliMeter - 20;
    if (distance > 0 && distance <= OBSTACLE_THRESHOLD) {
      turnAround();
      return;
    }
  }
  */

  int rawValues[numSensors];


  bool isBlack[numSensors];

  int sum = 0;
  int count = 0;

  // Read sensors and calculate Weighted Average Position
  for (int i = 0; i < numSensors; i++) {
    rawValues[i] = analogRead(sensorPins[i]);
    
    // Black line threshold
    if (rawValues[i] <= (minValues[i] + 400)) { 
      isBlack[i] = true;
      sum += i * 1000;
      count++;
    } else {
      isBlack[i] = false;
    }
  }

  // --- CHECK FOR FINISH LINE ---
  if (count == numSensors) {
    setMotorLeft(0);
    setMotorRight(0);
    Serial.println("All 8 sensors detected black! STOPPING...");
    while(true) {
      delay(100); 
    }
  }

  int error = 0;
  if (count > 0) {
    int position = sum / count; // Value from 0 to 7000
    
    // Center is 3500. 
    // If position > 3500 -> Line is on the left (towards sensor 7)
    // If position < 3500 -> Line is on the right (towards sensor 0)
    error = position - 3500;
  } else {
    // Lost the line. Remember the last error to keep turning to find it.
    if (lastError > 0) {
      error = 3500; // Turn hard left
    } else if (lastError < 0) {
      error = -3500; // Turn hard right
    } else {
      error = 0;
    }
  }

  // PID Algorithm
  float P = error;
  float D = error - lastError;
  int motorPwm = P * Kp + D * Kd; // Ignore Ki for simplicity
  lastError = error;

  // --- TỐC ĐỘ LINH HOẠT (DYNAMIC SPEED BOOST) ---
  int currentBaseSpeed;
  if (abs(error) < 400 && count > 0) {
    // Max speed on straights
    currentBaseSpeed = 250; 
  } else {
    // Controlled speed on curves
    currentBaseSpeed = baseSpeed; 
  }


  // Tính toán tốc độ motor dựa trên tốc độ cơ bản hiện tại và giá trị PID (motorPwm)
  int speedLeft = currentBaseSpeed - motorPwm;
  int speedRight = currentBaseSpeed + motorPwm;

  // Constrain speeds. Allowing negative speeds lets the car turn in place (sharp corners)
  speedLeft = constrain(speedLeft, -255, 255);
  speedRight = constrain(speedRight, -255, 255);


  setMotorLeft(speedLeft);
  setMotorRight(speedRight);

  // Minimal delay for maximum sampling rate
  delay(1);
}