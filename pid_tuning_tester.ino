#include <Arduino.h>
#include <ESP32Encoder.h>

/* ========== PINS ========== */
#define L_PWM 14
#define L_DIR 22
#define R_PWM 13
#define R_DIR 23
#define ENC_A 25
#define ENC_B 26

/* ========== PID VARIABLES ========== */
float Kp = 12.0, Ki =0.7, Kd = 600.0; 
float error, lastError, integral;
float target = 30.0; 
bool running = false;

/* ========== CALIBRATION ========== */
const float COUNTS_PER_CM = 530.52; 
ESP32Encoder encoder;

void drive(int speed, bool dir) {
  int pwm = constrain(speed, 0, 255); 
  if (pwm < 45 && pwm > 2) pwm = 45; 
  if (pwm <= 2) pwm = 0;

  digitalWrite(L_DIR, dir);
  digitalWrite(R_DIR, dir);
  analogWrite(L_PWM, pwm);
  analogWrite(R_PWM, pwm);
}

void setup() {
  Serial.begin(115200);
  pinMode(L_PWM, OUTPUT); pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_DIR, OUTPUT);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(26, 25); 
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    float val = Serial.parseFloat();
    if (cmd == 'p') Kp = val;
    else if (cmd == 'i') Ki = val;
    else if (cmd == 'd') Kd = val;
    else if (cmd == 't') target = val;
    else if (cmd == 's') { 
      running = true; 
      encoder.clearCount(); 
      integral = 0; 
      lastError = target; 
    }
    else if (cmd == 'r') { running = false; drive(0, HIGH); }

    // This matches your previous layout: Settings on one line
    Serial.print("Settings: Kp="); Serial.print(Kp);
    Serial.print(", Ki="); Serial.print(Ki);
    Serial.print(", Kd="); Serial.print(Kd);
    Serial.print(" | Target: "); Serial.println(target);
  }

  if (running) {
    long count = (long)encoder.getCount();
    float distance = abs(count / COUNTS_PER_CM);
    error = target - distance;

    if (abs(error) < 0.5) {
        integral += error;
    } else {
        integral = 0; 
    }
    integral = constrain(integral, -20, 20); 
    
    float derivative = error - lastError;
    lastError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Speed caps for no-overshoot
    if (error < 5.0 && error > 0) output = constrain(output, -50, 50); 
    if (error < 1.0 && error > 0) output = constrain(output, -35, 35); 

    if (abs(error) < 0.05 && abs(derivative) < 0.01) {
      drive(0, HIGH);
      running = false;
      Serial.print(">> TARGET REACHED. Final Dist: ");
      Serial.println(distance);
    } else {
      if (output >= 0) drive(abs(output), HIGH);
      else drive(abs(output), LOW); 
    }

    // Comma-separated output for Serial Plotter/Monitor
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 50) {
      Serial.print(target); 
      Serial.print(",");
      Serial.println(distance);
      lastPrint = millis();
    }
  }
}