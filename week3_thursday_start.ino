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

const int nodeConnection[7][7] = {
  {0, 0, 0, 0, 1, 1, 0},
  {0, 0, 0, 0, 0, 1, 1},
  {0, 0, 0, 1, 0, 1, 0},
  {0, 0, 1, 0, 0, 0, 1},
  {1, 0, 0, 1, 0, 0, 1},
  {1, 1, 1, 0, 0, 0, 1},
  {0, 1, 0, 1, 1, 0, 0}
};

bool nodeObstruction[7][7] = {
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0}
};

float position;
float error;
float integral = 0;
float derivative;
float correction;
float lastError = 0;

int MAX_SPEED = 255;
int BASE_SPEED = 120;

int distance_sensor = 16;
int distance = 0;

bool nodeDetected = false;
bool parked = false;
int nodes = 0;

int currentIndex = 0;

void stop_motor() {
  analogWrite(motorR_PWM,0);
  analogWrite(motorL_PWM,0); 
  Serial.println("Motors stopped");
  delay(500);
}

void turnLeft() {
  digitalWrite(motorR_phase,LOW);
  digitalWrite(motorL_phase,HIGH);
  analogWrite(motorR_PWM, 150);
  analogWrite(motorL_PWM,150);
  Serial.println("turning");
  delay(450);
}

void turnRight() {
  digitalWrite(motorR_phase,HIGH);
  digitalWrite(motorL_phase,LOW);
  analogWrite(motorR_PWM, 150);
  analogWrite(motorL_PWM,150);
  Serial.println("turning");
  delay(450);
}

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
  Serial.print(" ");
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

void obstacle() {
  distance = analogRead(distance_sensor);
  if (distance > 2800) {
    stop_motor();
    turnLeft();
    turnLeft();
  }
}

void obstacleParked() {
  distance = analogRead(distance_sensor);
  if (distance > 3200) {
    parked = true;
    stop_motor();
  }
}

void followCorrection() {  
  obstacle();
  
  bool lineDetected = false;

  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    AnalogValueMinus[i] = 5000 - AnalogValue[i];
    if (AnalogValueMinus[i] > 2200) {
      lineDetected = true;
      break;
    }
    else {
      lineDetected = false;
    }
  }

  if (lineDetected) {
    lineFollowPID();
  } 
  else {
    digitalWrite(motorR_phase, HIGH);
    digitalWrite(motorL_phase, HIGH);
    setMotorSpeed(100, 100);
    delay(300);
    setMotorSpeed(0, 0);
    delay(100);
  }

  debugPID();
}

void nodeDetection() {
  nodeDetected = false;
  
  int totalValue = 0;

  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    AnalogValueMinus[i] = 5000 - AnalogValue[i];
    totalValue += AnalogValueMinus[i];
  }

  if (totalValue > 15800) {
      nodeDetected = true;
      nodes++;
  }
  else {
    nodeDetected = false;
  }

  Serial.print(" Total Sensor Value: ");
  Serial.print(totalValue);
}

void parking() {
  nodeDetection();
  parked = false;
  if (nodeDetected == true) {
    setMotorSpeed(150, 150);
    while (parked == false) {
      obstacleParked();
      if (parked == true) {
        setMotorSpeed(0, 0);
      }
    }
  }
}

enum TurnDirection {
    LEFT,
    RIGHT,
    STRAIGHT,
    OPPOSITE,
    UNKNOWN
};

const int pathA[][2] = { {0, 4}, {4, 6}, {6, 3}, {3, 2}, {2, 5}, {5, 0} };
const int pathB[][2] = { {6, 1}, {1, 5} };

const int sizeA = sizeof(pathA) / sizeof(pathA[0]);
const int sizeB = sizeof(pathB) / sizeof(pathB[0]);

TurnDirection determineTurnAtDecisionPoint(int currNode, int nextNode, bool direction) {
    if (currNode == 6) {
        if (direction == 1) { // Clockwise
            if (nextNode == 1) return RIGHT;
            else if (nextNode == 4) return OPPOSITE;
            else if (nextNode == 3) return STRAIGHT;
        } else { // Counterclockwise
            if (nextNode == 1) return LEFT;
            else if (nextNode == 4) return STRAIGHT;
            else if (nextNode == 3) return OPPOSITE;
        }
    }

    if (currNode == 5) {
        if (direction == 1) { // Clockwise
            if (nextNode == 1) return RIGHT;
            else if (nextNode == 2) return OPPOSITE;
            else if (nextNode == 0) return STRAIGHT;
        } else { // Counterclockwise
            if (nextNode == 1) return LEFT;
            else if (nextNode == 2) return STRAIGHT;
            else if (nextNode == 0) return OPPOSITE;
        }
    }

    return STRAIGHT;
}

void getNextNode(int &currentIndex, int &currentArray, bool direction) {
    int (*currentPath)[2] = (currentArray == 0) ? pathA : pathB;
    int currentSize = (currentArray == 0) ? sizeA : sizeB;

    if (direction) { // Moving forward
        if (currentIndex < currentSize - 1) {
            currentIndex++; // Move forward to the next node
        } else {
            // Loop back to the start instead of resetting
            currentIndex = 0; 
        }
    } else { // Moving backward
        if (currentIndex > 0) {
            currentIndex--; // Move backward to the previous node
        } else {
            // Loop to the last node instead of resetting
            currentIndex = currentSize - 1;
        }
    }
}

void setup() {
  for (int i = 0; i < 5; i++) {
    pinMode(AnalogPin[i], INPUT);
  }
  pinMode(motorR_PWM, OUTPUT);
  pinMode(motorR_phase, OUTPUT);
  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorL_phase, OUTPUT);

  pinMode(distance_sensor,INPUT);
  
  Serial.begin(9600);
}

const int route[] = {0, 4, 6, 1, 5, 2, 3, 6, 4, 0, 5, 1};

void loop() {
    followCorrection();
    nodeDetection();
    if (nodeDetected) {
        TurnDirection turn = determineTurnAtDecisionPoint();
        
        switch (turn) {
            case LEFT:
                turnLeft();
                break;
            case RIGHT:
                turnRight();
                break;
            case STRAIGHT:
                setMotorSpeed(150, 150);
                delay(200);
                break;
        }
        
        nodeDetected = false;
        getNextNode(currentIndex, route, true);
    }
    delay(2);
}
