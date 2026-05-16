#include <Arduino.h>

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

// --- PID PARAMETERS ---
// Tweak these to make the car run smoother. 
// Start with small Kp, then increase. Finally add some Kd to reduce oscillation.
// Hạ thấp Kp và Kd vì động cơ 1000rpm rất mạnh, tránh xe bị lắc (oscillation)
float Kp = 0.08; // Hệ số tỉ lệ (Giảm từ 0.04 xuống 0.02)
float Kd = 0.4;  // Hệ số vi phân (Giảm từ 0.2 xuống 0.1)
float Ki = 0.0;

int baseSpeed = 120; // Base speed (0 - 255)
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

void setup() {
  Serial.begin(115200);
  Serial.println("=== KHOI DONG ROBOT DO LINE ===");
  
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  setupMotors();
  
  // Wait 2 seconds before start moving
  delay(2000); 
}

void loop() {
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
    // Khi xe đang đi thẳng (lỗi nhỏ < 400), tăng tốc độ lên để chạy nhanh hơn
    currentBaseSpeed = 180; // Bạn có thể tăng lên 200-255 nếu muốn nhanh hơn nữa
  } else {
    // Khi xe đang vào cua hoặc lệch nhiều, dùng tốc độ cơ bản thấp để an toàn
    currentBaseSpeed = baseSpeed; 
  }

  // Tính toán tốc độ motor dựa trên tốc độ cơ bản hiện tại và giá trị PID (motorPwm)
  int speedLeft = currentBaseSpeed - motorPwm;
  int speedRight = currentBaseSpeed + motorPwm;

  // Constrain speeds. Allowing negative speeds lets the car turn in place (sharp corners)
  speedLeft = constrain(speedLeft, -150, 255);
  speedRight = constrain(speedRight, -150, 255);

  // Apply to motors
  setMotorLeft(speedLeft);
  setMotorRight(speedRight);

  // Debug (uncomment to see the values in serial monitor)
  /*
  Serial.print("Pos: "); Serial.print(count > 0 ? sum/count : -1);
  Serial.print(" | Err: "); Serial.print(error);
  Serial.print(" | L: "); Serial.print(speedLeft);
  Serial.print(" | R: "); Serial.println(speedRight);
  */
  
  // Sample every 10ms (~100Hz)
  delay(10);
}