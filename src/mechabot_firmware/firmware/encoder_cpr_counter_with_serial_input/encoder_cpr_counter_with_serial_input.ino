//LEFT Motor
#define ENA 15
#define IN1 2
#define IN2 4

#define ENCODER_A_PIN 17
#define ENCODER_B_PIN 16

//RIGHT Motor
// #define ENA 19
// #define IN1 5
// #define IN2 18

// #define ENCODER_A_PIN 23
// #define ENCODER_B_PIN 22

volatile long encoder_ticks = 0;

long target_ticks = 0;
int speed = 100;

bool motor_running = false;

void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  Serial.println("Enter command: pX or nX  (example: p5, n3)");
}

void loop() {

  /* ---- WAIT FOR SERIAL COMMAND ---- */

  if (!motor_running && Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    char direction = cmd.charAt(0);
    int rotations = cmd.substring(1).toInt();

    target_ticks = rotations * 1000;

    encoder_ticks = 0;

    if (direction == 'p') {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      Serial.println("CW rotation started");
    }

    if (direction == 'n') {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      Serial.println("CCW rotation started");
    }

    analogWrite(ENA, speed);

    motor_running = true;
  }

  /* ---- RUN MOTOR UNTIL TARGET ---- */

  if (motor_running) {

    Serial.println(encoder_ticks);

    if (abs(encoder_ticks) >= target_ticks) {

      analogWrite(ENA, 0);

      motor_running = false;

      Serial.println("Motion Complete");
      Serial.println("Enter next command:");
    }
  }

  delay(10);
}


/* ---- ENCODER ISR ---- */

void encoderISR() {

  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN))
    encoder_ticks++;
  else
    encoder_ticks--;
}