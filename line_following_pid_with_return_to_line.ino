int AnalogValue[5] = {0, 0, 0, 0, 0};
int AnalogValueMinus[5] = {0, 0, 0, 0, 0};
int AnalogPin[5] = {4, 5, 6, 7, 15};

int motorL_PWM = 37;
int motorL_phase = 38;
int motorR_PWM = 39;
int motorR_phase = 20;

float Kp = 200;
float Ki = 0.01;
float Kd = 0.01;

float position;
float error;
float integral = 0;
float derivative;
float correction;
float lastError = 0;

int MAX_SPEED = 255;
int BASE_SPEED = 150;

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(motorR_PWM, constrain(rightSpeed, 0, MAX_SPEED));
  analogWrite(motorL_PWM, constrain(leftSpeed, 0, MAX_SPEED));
}

void lineFollowPID() {
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, LOW);
  
  int totalValue = 0;
  int weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    AnalogValueMinus[i] = 5000 - AnalogValue[i];
    totalValue += AnalogValueMinus[i];
    weightedSum += AnalogValueMinus[i] * (i + 1);
  }

  position = (totalValue > 0) ? (float)weightedSum / totalValue : 3;
  error = position - 3.0;
  integral += error;
  derivative = error - lastError;
  correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  int leftSpeed = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;
  setMotorSpeed(leftSpeed, rightSpeed);
}

void debugPID() {
  for (int i = 0; i < 5; i++) {
    Serial.print(AnalogValueMinus[i]);
    Serial.print(" ");
  }
  Serial.print(" Position: ");
  Serial.print(position);
  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print(" Correction: ");
  Serial.println(correction);
}

void setup() {
  for (int i = 0; i < 5; i++) {
    pinMode(AnalogPin[i], INPUT);
  }
  pinMode(motorR_PWM, OUTPUT);
  pinMode(motorR_phase, OUTPUT);
  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorL_phase, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  bool lineDetected = false;

  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    AnalogValueMinus[i] = 5000 - AnalogValue[i];
    if (AnalogValueMinus[i] > 3000) {
      lineDetected = true;
      break;
    }
    else {
      lineDetected = false;
    }
  }

  if (lineDetected) {
    lineFollowPID();
  } else {
    digitalWrite(motorR_phase, HIGH);
    digitalWrite(motorL_phase, HIGH);
    setMotorSpeed(100, 100);
    delay(500);
    setMotorSpeed(0, 0);
    delay(200);
  }

  debugPID();
  delay(2);
}