#include <Arduino.h>
#include <PID_v1.h>

// =====================================================
// L298N H-BRIDGE CONNECTION PINS
// =====================================================

// Left motor
#define L298N_enA 15   // Left motor PWM
#define L298N_in1 2    // Left motor direction
#define L298N_in2 4    // Left motor direction

// Right motor
#define L298N_in3 5    // Right motor direction
#define L298N_in4 18   // Right motor direction
#define L298N_enB 19   // Right motor PWM

// =====================================================
// WHEEL ENCODER CONNECTION PINS
// =====================================================

#define right_encoder_phaseA 23
#define right_encoder_phaseB 22

#define left_encoder_phaseA 17
#define left_encoder_phaseB 16

// =====================================================
// MOTOR / ENCODER SETTINGS
// =====================================================

#define TICKS_PER_REV 330.0
#define TWO_PI 6.28318530718

// =====================================================
// ESP32 PWM SETTINGS
// =====================================================

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

#define MIN_PWM 0
#define MAX_PWM 255

// =====================================================
// MOTOR TUNING FOR YOUR 280 RPM 12V MOTORS
// From your test:
// Left motor needs around PWM 82 for 100 RPM
// Right motor needs around PWM 70 for 100 RPM
// 100 RPM = 10.47 rad/s
// =====================================================

#define LEFT_PWM_PER_RAD_SEC 7.8
#define RIGHT_PWM_PER_RAD_SEC 6.7

#define LEFT_MIN_MOVING_PWM 35
#define RIGHT_MIN_MOVING_PWM 35

#define LEFT_START_PWM 160
#define RIGHT_START_PWM 150

#define STOP_THRESHOLD_RAD_S 0.01

// =====================================================
// TIMING
// =====================================================

const unsigned long interval = 100;  // ms
unsigned long last_millis = 0;

// =====================================================
// ENCODER COUNTERS
// =====================================================

volatile long right_encoder_ticks = 0;
volatile long left_encoder_ticks = 0;

// =====================================================
// SERIAL COMMAND PARSER
// Expected command format:
// rp05.30,lp05.30,
// rn05.30,ln05.30,
// =====================================================

bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;

bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;

char value[8];
uint8_t value_idx = 0;

// =====================================================
// PID VARIABLES
// =====================================================

// Setpoint: desired wheel speed magnitude in rad/s
double right_wheel_cmd_vel = 0.0;
double left_wheel_cmd_vel = 0.0;

// Input: measured wheel speed magnitude in rad/s
double right_wheel_meas_vel = 0.0;
double left_wheel_meas_vel = 0.0;

// Output: PID correction added to feedforward PWM
double right_pid_output = 0.0;
double left_pid_output = 0.0;

// PID gains
double Kp_r = 11.0;
double Ki_r = 2.5;
double Kd_r = 0.0;

double Kp_l = 11.0;
double Ki_l = 2.5;
double Kd_l = 0.0;

PID rightMotor(
  &right_wheel_meas_vel,
  &right_pid_output,
  &right_wheel_cmd_vel,
  Kp_r,
  Ki_r,
  Kd_r,
  DIRECT
);

PID leftMotor(
  &left_wheel_meas_vel,
  &left_pid_output,
  &left_wheel_cmd_vel,
  Kp_l,
  Ki_l,
  Kd_l,
  DIRECT
);

// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

void readSerialCommand();

void setLeftDirection(bool forward);
void setRightDirection(bool forward);

void stopMotors();
void writePWM(int pin, int pwm);

void resetValueBuffer();

void rightEncoderCallback();
void leftEncoderCallback();

// =====================================================
// SETUP
// =====================================================

void setup() {
  // Use 500000 for ROS 2 hardware interface.
  // For manual Arduino Serial Monitor testing, you may use 115200.
  Serial.begin(500000);

  // Motor direction pins
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Encoder pins
  pinMode(right_encoder_phaseA, INPUT_PULLUP);
  pinMode(right_encoder_phaseB, INPUT_PULLUP);
  pinMode(left_encoder_phaseA, INPUT_PULLUP);
  pinMode(left_encoder_phaseB, INPUT_PULLUP);

  // ESP32 PWM setup
  ledcAttach(L298N_enA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(L298N_enB, PWM_FREQ, PWM_RESOLUTION);

  // Default motor direction
  setLeftDirection(true);
  setRightDirection(true);

  // Encoder interrupts
  attachInterrupt(
    digitalPinToInterrupt(right_encoder_phaseA),
    rightEncoderCallback,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(left_encoder_phaseA),
    leftEncoderCallback,
    CHANGE
  );

  // PID setup
  rightMotor.SetOutputLimits(-80, 80);
  leftMotor.SetOutputLimits(-80, 80);

  rightMotor.SetSampleTime(interval);
  leftMotor.SetSampleTime(interval);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  resetValueBuffer();
  stopMotors();

  last_millis = millis();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  readSerialCommand();

  unsigned long current_millis = millis();

  if (current_millis - last_millis >= interval) {
    double dt = (current_millis - last_millis) / 1000.0;
    last_millis = current_millis;

    long right_ticks;
    long left_ticks;

    noInterrupts();
    right_ticks = right_encoder_ticks;
    left_ticks = left_encoder_ticks;
    right_encoder_ticks = 0;
    left_encoder_ticks = 0;
    interrupts();

    // Signed measured wheel velocity in rad/s
    double right_signed_vel =
      ((double)right_ticks / TICKS_PER_REV) * (TWO_PI / dt);

    double left_signed_vel =
      ((double)left_ticks / TICKS_PER_REV) * (TWO_PI / dt);

    // PID uses absolute magnitude
    right_wheel_meas_vel = fabs(right_signed_vel);
    left_wheel_meas_vel = fabs(left_signed_vel);

    int right_pwm = 0;
    int left_pwm = 0;

    // =====================================================
    // RIGHT MOTOR CONTROL
    // =====================================================

    if (right_wheel_cmd_vel <= STOP_THRESHOLD_RAD_S) {
      right_pid_output = 0.0;
      right_pwm = 0;
    } else {
      rightMotor.Compute();

      double right_ff_pwm = RIGHT_PWM_PER_RAD_SEC * right_wheel_cmd_vel;
      double right_total_pwm = right_ff_pwm + right_pid_output;

      // Starting boost
      if (right_wheel_meas_vel < 0.5 && right_total_pwm < RIGHT_START_PWM) {
        right_total_pwm = RIGHT_START_PWM;
      }

      // Minimum moving PWM
      if (right_total_pwm < RIGHT_MIN_MOVING_PWM) {
        right_total_pwm = RIGHT_MIN_MOVING_PWM;
      }

      right_pwm = constrain((int)right_total_pwm, MIN_PWM, MAX_PWM);
    }

    // =====================================================
    // LEFT MOTOR CONTROL
    // =====================================================

    if (left_wheel_cmd_vel <= STOP_THRESHOLD_RAD_S) {
      left_pid_output = 0.0;
      left_pwm = 0;
    } else {
      leftMotor.Compute();

      double left_ff_pwm = LEFT_PWM_PER_RAD_SEC * left_wheel_cmd_vel;
      double left_total_pwm = left_ff_pwm + left_pid_output;

      // Starting boost
      if (left_wheel_meas_vel < 0.5 && left_total_pwm < LEFT_START_PWM) {
        left_total_pwm = LEFT_START_PWM;
      }

      // Minimum moving PWM
      if (left_total_pwm < LEFT_MIN_MOVING_PWM) {
        left_total_pwm = LEFT_MIN_MOVING_PWM;
      }

      left_pwm = constrain((int)left_total_pwm, MIN_PWM, MAX_PWM);
    }

    // Write motor PWM
    writePWM(L298N_enA, left_pwm);
    writePWM(L298N_enB, right_pwm);

    // Feedback signs
    char right_sign = right_signed_vel >= 0.0 ? 'p' : 'n';
    char left_sign = left_signed_vel >= 0.0 ? 'p' : 'n';

    // Send wheel velocity feedback to ROS 2 hardware interface
    char buffer[40];

    snprintf(
      buffer,
      sizeof(buffer),
      "r%c%05.2f,l%c%05.2f,",
      right_sign,
      fabs(right_signed_vel),
      left_sign,
      fabs(left_signed_vel)
    );

    Serial.println(buffer);
  }
}

// =====================================================
// SERIAL COMMAND READER
// =====================================================

void readSerialCommand() {
  while (Serial.available()) {
    char chr = Serial.read();

    if (chr == 'r') {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      resetValueBuffer();
    }

    else if (chr == 'l') {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      resetValueBuffer();
    }

    else if (chr == 'p') {
      if (is_right_wheel_cmd) {
        setRightDirection(true);
        is_right_wheel_forward = true;
      } else if (is_left_wheel_cmd) {
        setLeftDirection(true);
        is_left_wheel_forward = true;
      }
    }

    else if (chr == 'n') {
      if (is_right_wheel_cmd) {
        setRightDirection(false);
        is_right_wheel_forward = false;
      } else if (is_left_wheel_cmd) {
        setLeftDirection(false);
        is_left_wheel_forward = false;
      }
    }

    else if (chr == ',') {
      double cmd = atof(value);

      if (is_right_wheel_cmd) {
        right_wheel_cmd_vel = cmd;
      } else if (is_left_wheel_cmd) {
        left_wheel_cmd_vel = cmd;
      }

      resetValueBuffer();
    }

    else {
      if (value_idx < sizeof(value) - 1) {
        value[value_idx] = chr;
        value_idx++;
        value[value_idx] = '\0';
      }
    }
  }
}

// =====================================================
// MOTOR DIRECTION FUNCTIONS
// =====================================================

void setLeftDirection(bool forward) {
  if (forward) {
    digitalWrite(L298N_in1, HIGH);
    digitalWrite(L298N_in2, LOW);
  } else {
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, HIGH);
  }
}

void setRightDirection(bool forward) {
  if (forward) {
    digitalWrite(L298N_in3, HIGH);
    digitalWrite(L298N_in4, LOW);
  } else {
    digitalWrite(L298N_in3, LOW);
    digitalWrite(L298N_in4, HIGH);
  }
}

void stopMotors() {
  writePWM(L298N_enA, 0);
  writePWM(L298N_enB, 0);

  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, LOW);
}

// =====================================================
// PWM FUNCTION
// =====================================================

void writePWM(int pin, int pwm) {
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  ledcWrite(pin, pwm);
}

// =====================================================
// VALUE BUFFER RESET
// =====================================================

void resetValueBuffer() {
  for (uint8_t i = 0; i < sizeof(value); i++) {
    value[i] = '\0';
  }

  value_idx = 0;
}

// =====================================================
// ENCODER CALLBACKS
// =====================================================

void rightEncoderCallback() {
  if (digitalRead(right_encoder_phaseB) == HIGH) {
    right_encoder_ticks--;
  } else {
    right_encoder_ticks++;
  }
}

void leftEncoderCallback() {
  if (digitalRead(left_encoder_phaseB) == HIGH) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}