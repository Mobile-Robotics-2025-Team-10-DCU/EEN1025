#include <Arduino.h>

// Analog sensor pins and values
int AnalogValue[5] = {0, 0, 0, 0, 0};
int AnalogValueMinus[5] = {0, 0, 0, 0, 0};
int AnalogPin[5] = {4, 5, 6, 7, 15};

// Motor pins
int motorL_PWM = 37;
int motorL_phase = 38;
int motorR_PWM = 39;
int motorR_phase = 20;

// PID constants
float Kp = 200;
float Ki = 0.01;
float Kd = 0.01;

float position, error, integral = 0, derivative, correction, lastError = 0;
int MAX_SPEED = 255;
int BASE_SPEED = 120;

// Distance sensor
int distance_sensor = 16;
int distance = 0;

// Navigation variables
bool nodeDetected = false;
int nodesVisited = 0;
int currentNode = -1;
int previousNode = -1;

//innovation motion start
bool motiondetection = false;
unsigned long motionStartTime = 0;  
const unsigned long motionThreshold = 1500;

void startmotion() {
  distance = analogRead(distance_sensor);
  if (distance > 2200) {
    motiondetection=true;
    } 
}

// Sample path (will be provided by server)
int path[] = {3, 6, 1, 5, 0, 5, 2};  // Example path
int pathLength;

// Node connection layout with distances
const int nodeConnection[7][7] = {
  {0, 0, 0, 0, 2, 1, 0},
  {0, 0, 0, 0, 0, 1, 1},
  {0, 0, 0, 1, 0, 1, 0},
  {0, 0, 1, 0, 0, 0, 1},
  {2, 0, 0, 1, 0, 0, 1},
  {1, 1, 1, 0, 0, 0, 1},
  {0, 1, 0, 1, 1, 0, 0}
};

// Define turn directions for each valid node connection
// Format: [current_node][from_node][to_node] = turn_direction
// Turn direction: 0 = straight, 1 = right, 2 = left, 3 = around, -1 = invalid
const int nodeTurns[7][7][7] = {
    // Node 0 (top)
    {{-1,-1,-1,-1,-1,-1,-1},    // from 0 (invalid)
     {-1,-1,-1,-1,-1,-1,-1},  // from 1
     {-1,-1,-1,-1,-1,-1,-1},  // from 2
     {-1,-1,-1,-1,-1,-1,-1},  // from 3
     {-1,-1,-1,-1,3,0,-1},   // from 4 (to 5=left)
     {-1,-1,-1,-1,0,3,-1},   // from 5
     {-1,-1,-1,-1,-1,-1,-1}},   // from 6

    // Node 1 (inner middle)
    {{-1,-1,-1,-1,-1,-1,-1},    // from 0
     {-1,-1,-1,-1,-1,-1,-1},  // from 1 (invalid)
     {-1,-1,-1,-1,-1,-1,-1},  // from 2
     {-1,-1,-1,-1,-1,-1,-1},  // from 3
     {-1,-1,-1,-1,-1,-1,-1},  // from 4
     {-1,-1,-1,-1,-1,3,0},   // from 5 (to 6=right)
     {-1,-1,-1,-1,-1,0,3}},  // from 6 (to 5=left)

    // Node 2 (bottom)
    {{-1,-1,-1,-1,-1,-1,-1},     // from 0
     {-1,-1,-1,-1,-1,-1,-1},  // from 1
     {-1,-1,-1,-1,-1,-1,-1},  // from 2 (invalid)
     {-1,-1,-1,-1,-1,0,-1},   // from 3 (to 5=left)
     {-1,-1,-1,-1,-1,-1,-1},  // from 4
     {-1,-1,-1,0,-1,-1,-1},   // from 5 (to 3=right)
     {-1,-1,-1,-1,-1,-1,-1}}, // from 6

    // Node 3 (bottom right)
    {{-1,-1,-1,-1,-1,-1,-1},    // from 0
     {-1,-1,-1,-1,-1,-1,-1},  // from 1
     {-1,-1,3,-1,-1,-1,0},   // from 2 (to 6=right)
     {-1,-1,-1,-1,-1,-1,-1},  // from 3 (invalid)
     {-1,-1,-1,-1,-1,-1,-1},  // from 4
     {-1,-1,-1,-1,-1,-1,-1},  // from 5
     {-1,-1,0,-1,-1,-1,3}},  // from 6 (to 2=left)

    // Node 4 (top right)
    {{3,-1,-1,-1,-1,-1,0},    // from 0 (to 6=right)
     {-1,-1,-1,-1,-1,-1,-1},  // from 1
     {-1,-1,-1,-1,-1,-1,-1},  // from 2
     {-1,-1,-1,-1,-1,-1,-1},  // from 3
     {-1,-1,-1,-1,-1,-1,-1},  // from 4 (invalid)
     {-1,-1,-1,-1,-1,-1,-1},  // from 5
     {0,-1,-1,-1,-1,-1,3}},  // from 6 (to 0=left)

    // Node 5 (left)
    {{3,2,0,-1,-1,-1,-1},    // from 0 (to 2=right)
     {1,3,2,-1,-1,-1,0},   // from 1 (to 6=straight)
     {0,1,3,-1,-1,-1,-1},   // from 2 (to 6=straight)
     {-1,-1,-1,-1,-1,-1,-1},  // from 3
     {-1,-1,-1,-1,-1,-1,-1},  // from 4
     {-1,-1,-1,-1,-1,-1,-1},  // from 5 (invalid)
     {-1,-1,-1,-1,-1,-1,-1}},   // from 6 (to 1=left, to 2=right)

    // Node 6 (right)
    {{-1,-1,-1,-1,-1,-1,-1},     // from 0 (to 3=straight, to 1=left)
     {-1,3,-1,1,2,-1,-1},     // from 1 (to 3/4/5=straight)
     {-1,-1,-1,-1,-1,-1,-1},  // from 2
     {-1,2,-1,3,0,-1,-1},     // from 3 (to 1=left, to 4=right)
     {-1,1,-1,0,3,-1,-1},     // from 4 (to 1=left, to 3=left)
     {-1,-1,-1,-1,-1,-1,-1},     // from 5 (to 3/4=straight, to 1=left)
     {-1,-1,-1,-1,-1,-1,-1}}  // from 6 (invalid)
};

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(motorR_PWM, constrain(rightSpeed, 0, MAX_SPEED));
  analogWrite(motorL_PWM, constrain(leftSpeed, 0, MAX_SPEED));
}

void stop_motor() {
  analogWrite(motorR_PWM, 0);
  analogWrite(motorL_PWM, 0);
  delay(500);
}

void turnLeft() {
  setMotorSpeed(150, 150);
  delay(300);
  digitalWrite(motorR_phase, HIGH);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 150);
  analogWrite(motorL_PWM, 145);
  delay(300);
}

void turnRight() {
  setMotorSpeed(150, 150);
  delay(300);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, HIGH);
  analogWrite(motorR_PWM, 145);
  analogWrite(motorL_PWM, 150);
  delay(300);
}

void turnAround() {
  setMotorSpeed(150, 150);
  delay(300);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, HIGH);
  analogWrite(motorR_PWM, 145);
  analogWrite(motorL_PWM, 150);
  delay(700);  // Double the time of a normal turn
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

void checkObstacle() {
  distance = analogRead(distance_sensor);
  if (distance > 2800) {
    stop_motor();
    turnAround();
  }
}

String getNextTurn(int current, int prev, int next) {
    // Get turn direction from the 3D matrix
    int turnDirection = nodeTurns[current][prev][next];
    
    switch(turnDirection) {
        case 0: return "straight";
        case 1: return "left";
        case 2: return "right";
        case 3: return "around";
        default: return "straight"; // Default case
    }
}

void handleNode() {
  if (nodesVisited >= pathLength) {
    stop_motor();
    while(1); // End of path
  }

  currentNode = path[nodesVisited];
  
  if (nodesVisited < pathLength - 1) {
    int nextNode = path[nodesVisited + 1];
    String turn = getNextTurn(currentNode, previousNode, nextNode);
    
    if (turn == "left") {
      turnLeft();
    } else if (turn == "right") {
      turnRight();
    } else if (turn == "around") {
      turnAround();
    } else {
      // Go straight
      setMotorSpeed(BASE_SPEED, BASE_SPEED);
      delay(300);
    }
  }
  
  previousNode = currentNode;
  nodesVisited++;
}

void detectNode() {
  int totalValue = 0;
  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    AnalogValueMinus[i] = 5000 - AnalogValue[i];
    totalValue += AnalogValueMinus[i];
  }

  if (totalValue > 15800 && !nodeDetected) {
    nodeDetected = true;
    handleNode();
    delay(200);  // Debounce delay
  } else if (totalValue <= 15800) {
    nodeDetected = false;
  }
}

void setup() {
  // Initialize pins
  for (int i = 0; i < 5; i++) pinMode(AnalogPin[i], INPUT);
  pinMode(motorR_PWM, OUTPUT);
  pinMode(motorR_phase, OUTPUT);
  pinMode(motorL_PWM, OUTPUT);
  pinMode(motorL_phase, OUTPUT);
  pinMode(distance_sensor, INPUT);
  
  // Initialize path length
  pathLength = sizeof(path) / sizeof(path[0]);
  
  Serial.begin(9600);
}

void loop() {
  startmotion();
  if(motiondetection == true){
    delay(800);
  }
  while(motiondetection == true){
  detectNode();
  lineFollowPID();
  checkObstacle();
  delay(2);
  }
}
