#include <Arduino.h>

// =====================================================
// MOTOR 1 / LEFT MOTOR
// =====================================================
#define M1_ENA 15
#define M1_IN1 2
#define M1_IN2 4

#define M1_ENCODER_A 17
#define M1_ENCODER_B 16

// =====================================================
// MOTOR 2 / RIGHT MOTOR
// =====================================================
#define M2_ENA 19
#define M2_IN1 5
#define M2_IN2 18

#define M2_ENCODER_A 23
#define M2_ENCODER_B 22

// =====================================================
// MOTOR / ENCODER SETTINGS
// =====================================================
#define TARGET_RPM 100.0
#define TICKS_PER_REV 330.0

// =====================================================
// PWM SETTINGS
// Use 1000 Hz because your RPM test worked with this
// =====================================================
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define MIN_PWM 0
#define MAX_PWM 255

// =====================================================
// CONTROL SETTINGS
// =====================================================
#define SAMPLE_TIME_MS 500

// Based on your motor test:
// PWM 200 gave M1 ≈ 262 RPM, M2 ≈ 286 RPM
// For 100 RPM, start around 80–90 PWM, but use strong stall boost.
float M1_BASE_PWM = 65.0;
float M2_BASE_PWM = 55.0;

float m1_pwm = M1_BASE_PWM;
float m2_pwm = M2_BASE_PWM;

float Kp1 = 1.2;
float Ki1 = 0.25;

float Kp2 = 1.2;
float Ki2 = 0.25;

float m1_integral = 0.0;
float m2_integral = 0.0;

float m1_rpm_filtered = 0.0;
float m2_rpm_filtered = 0.0;

float FILTER_ALPHA = 0.60;
float MAX_PWM_STEP = 10.0;

// Stronger start values
#define M1_START_PWM 160
#define M2_START_PWM 150

// Minimum moving PWM after startup
#define M1_MIN_MOVING_PWM 35
#define M2_MIN_MOVING_PWM 35

volatile long m1_ticks = 0;
volatile long m2_ticks = 0;

long prev_m1_ticks = 0;
long prev_m2_ticks = 0;

unsigned long prev_time = 0;

void motor1Forward();
void motor2Forward();
void motor1Stop();
void motor2Stop();

void m1EncoderISR();
void m2EncoderISR();

void writePWM(int pin, int pwm);
float rampPWM(float current_pwm, float target_pwm, float max_step);

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M1_ENCODER_A, INPUT_PULLUP);
  pinMode(M1_ENCODER_B, INPUT_PULLUP);
  pinMode(M2_ENCODER_A, INPUT_PULLUP);
  pinMode(M2_ENCODER_B, INPUT_PULLUP);

  ledcAttach(M1_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(M2_ENA, PWM_FREQ, PWM_RESOLUTION);

  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), m1EncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), m2EncoderISR, CHANGE);

  motor1Forward();
  motor2Forward();

  Serial.println("Starting motors with strong kick...");

  writePWM(M1_ENA, M1_START_PWM);
  writePWM(M2_ENA, M2_START_PWM);
  delay(800);

  m1_pwm = M1_BASE_PWM;
  m2_pwm = M2_BASE_PWM;

  writePWM(M1_ENA, (int)m1_pwm);
  writePWM(M2_ENA, (int)m2_pwm);

  prev_time = millis();

  Serial.println("=======================================");
  Serial.println("Closed-loop 2 Motor RPM Controller");
  Serial.println("PWM Frequency: 1000 Hz");
  Serial.print("Target RPM: ");
  Serial.println(TARGET_RPM);
  Serial.println("=======================================");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  unsigned long now = millis();

  if (now - prev_time >= SAMPLE_TIME_MS) {
    float dt = (now - prev_time) / 1000.0;
    prev_time = now;

    long current_m1_ticks;
    long current_m2_ticks;

    noInterrupts();
    current_m1_ticks = m1_ticks;
    current_m2_ticks = m2_ticks;
    interrupts();

    long delta_m1 = current_m1_ticks - prev_m1_ticks;
    long delta_m2 = current_m2_ticks - prev_m2_ticks;

    prev_m1_ticks = current_m1_ticks;
    prev_m2_ticks = current_m2_ticks;

    float m1_rpm_raw = (abs(delta_m1) / TICKS_PER_REV) * (60.0 / dt);
    float m2_rpm_raw = (abs(delta_m2) / TICKS_PER_REV) * (60.0 / dt);

    m1_rpm_filtered =
      (FILTER_ALPHA * m1_rpm_filtered) + ((1.0 - FILTER_ALPHA) * m1_rpm_raw);

    m2_rpm_filtered =
      (FILTER_ALPHA * m2_rpm_filtered) + ((1.0 - FILTER_ALPHA) * m2_rpm_raw);

    float m1_error = TARGET_RPM - m1_rpm_filtered;
    float m2_error = TARGET_RPM - m2_rpm_filtered;

    m1_integral += m1_error * dt;
    m2_integral += m2_error * dt;

    m1_integral = constrain(m1_integral, -300.0, 300.0);
    m2_integral = constrain(m2_integral, -300.0, 300.0);

    float m1_desired_pwm =
      M1_BASE_PWM + (Kp1 * m1_error) + (Ki1 * m1_integral);

    float m2_desired_pwm =
      M2_BASE_PWM + (Kp2 * m2_error) + (Ki2 * m2_integral);

    // If motor is not moving, apply strong boost
    if (m1_rpm_raw < 3.0 && TARGET_RPM > 0) {
      m1_desired_pwm = M1_START_PWM;
    }

    if (m2_rpm_raw < 3.0 && TARGET_RPM > 0) {
      m2_desired_pwm = M2_START_PWM;
    }

    // Do not allow too-low PWM while target is active
    if (TARGET_RPM > 0) {
      m1_desired_pwm = max(m1_desired_pwm, (float)M1_MIN_MOVING_PWM);
      m2_desired_pwm = max(m2_desired_pwm, (float)M2_MIN_MOVING_PWM);
    }

    m1_desired_pwm = constrain(m1_desired_pwm, 0.0, 255.0);
    m2_desired_pwm = constrain(m2_desired_pwm, 0.0, 255.0);

    m1_pwm = rampPWM(m1_pwm, m1_desired_pwm, MAX_PWM_STEP);
    m2_pwm = rampPWM(m2_pwm, m2_desired_pwm, MAX_PWM_STEP);

    writePWM(M1_ENA, (int)m1_pwm);
    writePWM(M2_ENA, (int)m2_pwm);

    Serial.print("M1 Raw RPM: ");
    Serial.print(m1_rpm_raw);
    Serial.print(" | M1 Filtered RPM: ");
    Serial.print(m1_rpm_filtered);
    Serial.print(" | M1 PWM: ");
    Serial.print(m1_pwm);

    Serial.print(" || M2 Raw RPM: ");
    Serial.print(m2_rpm_raw);
    Serial.print(" | M2 Filtered RPM: ");
    Serial.print(m2_rpm_filtered);
    Serial.print(" | M2 PWM: ");
    Serial.println(m2_pwm);
  }
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================
void motor1Forward() {
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
}

void motor2Forward() {
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);
}

void motor1Stop() {
  writePWM(M1_ENA, 0);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
}

void motor2Stop() {
  writePWM(M2_ENA, 0);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
}

// =====================================================
// ENCODER INTERRUPTS
// =====================================================
void m1EncoderISR() {
  if (digitalRead(M1_ENCODER_A) == digitalRead(M1_ENCODER_B)) {
    m1_ticks++;
  } else {
    m1_ticks--;
  }
}

void m2EncoderISR() {
  if (digitalRead(M2_ENCODER_A) == digitalRead(M2_ENCODER_B)) {
    m2_ticks++;
  } else {
    m2_ticks--;
  }
}

// =====================================================
// HELPER FUNCTIONS
// =====================================================
void writePWM(int pin, int pwm) {
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  ledcWrite(pin, pwm);
}

float rampPWM(float current_pwm, float target_pwm, float max_step) {
  if (target_pwm > current_pwm + max_step) {
    return current_pwm + max_step;
  } else if (target_pwm < current_pwm - max_step) {
    return current_pwm - max_step;
  } else {
    return target_pwm;
  }
}