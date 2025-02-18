#include <Arduino.h>
#include <WiFi.h>

// WiFi details
char ssid[] = "iot";
char password[] = "nonusurpingly11barkentines";

char server[] = "3.250.38.184";
int port = 8000;

WiFiClient client;

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
int nodesVisited = 1;
int currentNode = 4;
int previousNode = 6;  // Robot starts from direction of node 4
int previousNode2;
bool firstNode = true;  // Flag to handle first node specially

// Path planning variables
int optimizedPath[21];  // Will store the complete optimized path
int optimizedPathLength;
bool pathOptimized = false;
int target;

bool parkingS = 0;

//innovation
#define SERVO_PIN 1 

// Node connection layout with distances
int nodeConnection[7][7] = {
  {0, 0, 0, 0, 2, 2, 0},
  {0, 0, 0, 0, 0, 2, 1},
  {0, 0, 0, 2, 0, 2, 0},
  {0, 0, 2, 0, 0, 0, 4},
  {2, 0, 0, 0, 0, 0, 4},
  {2, 2, 2, 0, 0, 0, 0},
  {0, 1, 0, 4, 4, 0, 0}
};

// Define turn directions for each valid node connection
// Format: [current_node][from_node][to_node] = turn_direction
// Turn direction: 0 = straight, 1 = right, 2 = left, 3 = around, -1 = invalid
const int nodeTurns[7][7][7] = {
    // Node 0 (top)
    {{-1,-1,-1,-1,-1,-1,-1},  // from 0
     {-1,-1,-1,-1,-1,-1,-1},  // from 1
     {-1,-1,-1,-1,-1,-1,-1},  // from 2
     {-1,-1,-1,-1,-1,-1,-1},  // from 3
     {-1,-1,-1,-1,3,0,-1},    // from 4
     {-1,-1,-1,-1,0,3,-1},    // from 5
     {-1,-1,-1,-1,-1,-1,-1}}, // from 6

    // Node 1 (inner middle)
    {{-1,-1,-1,-1,-1,-1,-1},  // from 0
     {-1,-1,-1,-1,-1,-1,-1},  // from 1 (invalid)
     {-1,-1,-1,-1,-1,-1,-1},  // from 2
     {-1,-1,-1,-1,-1,-1,-1},  // from 3
     {-1,-1,-1,-1,-1,-1,-1},  // from 4
     {-1,-1,-1,-1,-1,3,0},    // from 5
     {-1,-1,-1,-1,-1,0,3}},   // from 6

    // Node 2 (bottom)
    {{-1,-1,-1,-1,-1,-1,-1},     // from 0
     {-1,-1,-1,-1,-1,-1,-1},  // from 1
     {-1,-1,-1,-1,-1,-1,-1},  // from 2 (invalid)
     {-1,-1,-1,3,-1,0,-1},   // from 3 (to 5=left)
     {-1,-1,-1,-1,-1,-1,-1},  // from 4
     {-1,-1,-1,0,-1,3,-1},   // from 5 (to 3=right)
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
     {1,3,2,-1,-1,-1,-1},   // from 1 (to 6=straight)
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

// Function to find the vertex with minimum distance
int minDistance(int dist[], bool visited[], int V) {
    int min = INT_MAX;
    int min_index = -1;
    
    for (int v = 0; v < V; v++) {
        if (!visited[v] && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }
    return min_index;
}

// Function to get the path from source to destination
void getPath(int parent[], int dest, int path[], int& pathLength) {
    if (parent[dest] == -1) {
        path[0] = dest;
        pathLength = 1;
        return;
    }
    
    getPath(parent, parent[dest], path, pathLength);
    path[pathLength] = dest;
    pathLength++;
}

// Function to find the shortest path between two nodes
void findShortestPath(int start, int end, int path[], int& pathLength) {
    const int V = 7; // Number of vertices
    int dist[V];     // Distance array
    bool visited[V]; // Visited array
    int parent[V];   // Parent array to reconstruct path
    
    // Initialize arrays
    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        visited[i] = false;
        parent[i] = -1;
    }
    
    dist[start] = 0;
    
    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, visited, V);
        visited[u] = true;
        
        for (int v = 0; v < V; v++) {
            if (!visited[v] && nodeConnection[u][v] && 
                dist[u] != INT_MAX && 
                dist[u] + nodeConnection[u][v] < dist[v]) {
                dist[v] = dist[u] + nodeConnection[u][v];
                parent[v] = u;
            }
        }
    }
    
    // Get the path
    pathLength = 0;
    getPath(parent, end, path, pathLength);
}

// Function to calculate total distance of a path
int calculatePathDistance(int path[], int pathLength) {
    int totalDistance = 0;
    for (int i = 0; i < pathLength - 1; i++) {
        totalDistance += nodeConnection[path[i]][path[i + 1]];
    }
    return totalDistance;
}

// Function to find optimal route through multiple waypoints
void findOptimalRoute(int waypoints[], int numWaypoints, int finalPath[], int& finalPathLength) {
    finalPathLength = 0;
    int tempPath[7];  // Temporary array for each segment
    int tempLength;
    
    // Process each consecutive pair of waypoints
    for (int i = 0; i < numWaypoints - 1; i++) {
        // Find shortest path between current waypoint pair
        findShortestPath(waypoints[i], waypoints[i + 1], tempPath, tempLength);
        
        // Add path to final route (skip last node except for final segment to avoid duplicates)
        if (i < numWaypoints - 2) {
            tempLength--; // Don't include the last node as it will be the start of next segment
        }
        
        // Copy segment to final path
        for (int j = 0; j < tempLength; j++) {
            finalPath[finalPathLength++] = tempPath[j];
        }
    }
}

// Function to print the route details
void printRoute(int path[], int pathLength) {
    Serial.println("Optimal Route:");
    for (int i = 0; i < pathLength; i++) {
        Serial.print(path[i]);
        if (i < pathLength - 1) {
            Serial.print(" -> ");
        }
    }
    Serial.println();
    Serial.print("Total Distance: ");
    Serial.println(calculatePathDistance(path, pathLength));
}

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
  setMotorSpeed(0, 0);
  delay(200);
  digitalWrite(motorR_phase, HIGH);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 150);
  analogWrite(motorL_PWM, 145);
  delay(450);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 0);
  analogWrite(motorL_PWM, 0);
}

void turnRight() {
  setMotorSpeed(150, 150);
  delay(300);
  setMotorSpeed(0, 0);
  delay(200);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, HIGH);
  analogWrite(motorR_PWM, 145);
  analogWrite(motorL_PWM, 150);
  delay(450);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 0);
  analogWrite(motorL_PWM, 0);
}

void turnPLeft() {
  setMotorSpeed(150, 150);
  delay(300);
  setMotorSpeed(0, 0);
  delay(200);
  digitalWrite(motorR_phase, HIGH);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 150);
  analogWrite(motorL_PWM, 145);
  delay(450);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 0);
  analogWrite(motorL_PWM, 0);
}

void turnPRight() {
  setMotorSpeed(150, 150);
  delay(300);
  setMotorSpeed(0, 0);
  delay(200);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, HIGH);
  analogWrite(motorR_PWM, 145);
  analogWrite(motorL_PWM, 150);
  delay(450);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 0);
  analogWrite(motorL_PWM, 0);
}

void turnAround() {
  setMotorSpeed(0, 0);
  delay(200);
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, HIGH);
  analogWrite(motorR_PWM, 145);
  analogWrite(motorL_PWM, 150);
  delay(1000);  // Double the time of a normal turn
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, LOW);
  analogWrite(motorR_PWM, 0);
  analogWrite(motorL_PWM, 0);
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

void writeServo(int angle){
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);
  delay(10);
}

void runServo(){
  delay(500);
  writeServo(90);
  delay(500);
  // Move from 90° to 180° (Right)
  for (int pos = 90; pos <= 180; pos++) {
    writeServo(pos);
    delay(15);
  }
  delay(100); 
  // Move from 180° (Right) back to 90° 
  for (int pos = 180; pos >= 90; pos--) {
    writeServo(pos);
    delay(15);
  }
  delay(500); 
  // Move from 90° to 0° (Left)
  for (int pos = 90; pos >= 0; pos--) {
    writeServo(pos);
    delay(15);
  }
  delay(100); 
  // Move from 0° back to 90°
  for (int pos = 0; pos<=90; pos++) {
    writeServo(pos);
    delay(15);
  }
  delay(500);
}

void checkObstacle() {
  distance = analogRead(distance_sensor);
  if (distance > 3000) {
    stop_motor();
    digitalWrite(motorR_phase, HIGH);
    digitalWrite(motorL_phase, HIGH);
    analogWrite(motorR_PWM, 150);
    analogWrite(motorL_PWM, 150);
    delay(200);

    // Block the current path segment
    nodeConnection[previousNode][currentNode] = 0;
    nodeConnection[currentNode][previousNode] = 0;
    
    turnAround();
    
    // Swap current and previous nodes after turning around
    int tempNode = currentNode;
    currentNode = previousNode;
    previousNode = tempNode;
    
    // Recalculate path from new current node to target
    int newPath[21];
    newPath[0] = currentNode;
    newPath[1] = target;
    findOptimalRoute(newPath, 2, optimizedPath, optimizedPathLength);
    pathOptimized = true;
    
    // Reset navigation state
    nodesVisited = 0;
    firstNode = true;
    
    Serial.println("Rerouting due to obstacle. New path:");
    printRoute(optimizedPath, optimizedPathLength);

    setMotorSpeed(0, 0);
    runServo();
  }
}

void returnToLine() {
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
    if (!pathOptimized) {
        stop_motor();
        while(1);
        return;
    }

    // Print debug information for every node detection
    Serial.print("Node detected! Current node: ");
    Serial.print(currentNode);
    Serial.print(", Previous node: ");
    Serial.print(previousNode);
    Serial.print(", Nodes visited: ");
    Serial.println(nodesVisited);

    if (firstNode) {
        // Special handling for first node
        firstNode = false;
        
        // Get next node from optimized path
        int nextNode = optimizedPath[1];
        
        String turn = getNextTurn(currentNode, previousNode, nextNode);
        Serial.print("First node turn direction: ");
        Serial.println(turn);
        
        if ((parkingS == 1) && (currentNode == 6)) {
          parking();
        }

        if (turn == "left") {
            turnLeft();
        } else if (turn == "right") {
            turnRight();
        } else if (turn == "around") {
            turnAround();
        }
        else {
        // Go straight
          setMotorSpeed(BASE_SPEED, BASE_SPEED);
          delay(200);
        }
        
        delay(200);

        previousNode2 = previousNode;
        previousNode = currentNode;  // Update previous node
        currentNode = nextNode;      // Update current node to where we're heading
        nodesVisited = 1;
        return;
    }

    if (nodesVisited >= optimizedPathLength - 1) {
        stop_motor();
        while(1); // End of path
        return;
    }

    // Get next node from the path
    int nextNode = optimizedPath[nodesVisited + 1];
    
    // Get the turn direction based on current position and next node
    String turn = getNextTurn(currentNode, previousNode, nextNode);
    
    Serial.print("At node: ");
    Serial.print(currentNode);
    Serial.print(" from: ");
    Serial.print(previousNode);
    Serial.print(" to: ");
    Serial.print(nextNode);
    Serial.print(" turn: ");
    Serial.println(turn);

    if ((parkingS == 1) && (currentNode == 6)) {
      parking();
    }

    // Execute the turn
    if (turn == "left") {
        turnLeft();
    } else if (turn == "right") {
        turnRight();
    } else if (turn == "around") {
        turnAround();
    } else {
        // Go straight
        setMotorSpeed(BASE_SPEED, BASE_SPEED);
        delay(200);
    }
    
    delay(200);

    previousNode2 = previousNode;
    previousNode = currentNode;      // Update previous node before updating current
    currentNode = nextNode;          // Update current node to where we're heading
    nodesVisited++;
}

void detectNode() {
  int totalValue = 0;
  for (int i = 0; i < 5; i++) {
    AnalogValue[i] = analogRead(AnalogPin[i]);
    AnalogValueMinus[i] = 5000 - AnalogValue[i];
    totalValue += AnalogValueMinus[i];
  }
  if (totalValue > 17500 && !nodeDetected) {
    nodeDetected = true;
    handleNode();
    delay(2);  // Debounce delay
  } else if (totalValue <= 17500) {
    nodeDetected = false;
  }
}

void parking() {
  if (previousNode == 1) {
    while (true) {
      setMotorSpeed(150, 145);
      distance = analogRead(distance_sensor);
      Serial.print(distance);
      if (distance > 3000) {
        stop_motor();
        connectToWiFi();
        sendPostRequest(currentNode);
        while(1);
        return;
      }
    }
  }
  if (previousNode == 3) {
    turnPLeft();
    while (true) {
      setMotorSpeed(150, 145);
      distance = analogRead(distance_sensor);
      Serial.print(distance);
      if (distance > 3000) {
        stop_motor();
        connectToWiFi();
        sendPostRequest(currentNode);
        while(1);
        return;
      }
    }
  }
  if (previousNode == 4) {
    turnPRight();
    while (true) {
      setMotorSpeed(150, 145);
      distance = analogRead(distance_sensor);
      Serial.print(distance);
      if (distance > 3000) {
        stop_motor();
        connectToWiFi();
        sendPostRequest(currentNode);
        while(1);
        return;
      }
    }
  }
}

// WiFi ---------------------------------------------------------------------------------

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(2);
  }
  Serial.println(WiFi.localIP());
}

void sendPostRequest(int position) {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  String postBody = "position=" + String(position);
  Serial.println(port);
  if (client.connect(server, port)) {
    client.println("POST /api/arrived/dggb1873 HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(postBody.length());
    client.println();
    client.println(postBody);
  }
}

String readResponse() {
  String response = "";
  unsigned long timeout = millis() + 50;
  while (client.connected() && millis() < timeout) {
    while (client.available()) {
      response += (char)client.read();
    }
  }
  if (response == "") {
    Serial.println("No response");
  }
  return response;
}

String getResponseBody(String& response) {
  int split = response.indexOf("\r\n\r\n");
  if (split == -1) {
    return "";
  }
  String body =  response.substring(split + 4);
  body.trim();
  return body;
}

void setup() {
    // Initialize pins (unchanged)
    for (int i = 0; i < 5; i++) pinMode(AnalogPin[i], INPUT);
    pinMode(motorR_PWM, OUTPUT);
    pinMode(motorR_phase, OUTPUT);
    pinMode(motorL_PWM, OUTPUT);
    pinMode(motorL_phase, OUTPUT);
    pinMode(distance_sensor, INPUT);
    pinMode(SERVO_PIN, OUTPUT);
    
    Serial.begin(9600);

    bool begin = true;
    
    connectToWiFi();
    String response = readResponse();
    String body = getResponseBody(response);
    target = body.toInt();
    client.stop();
    
    int newPath[21];
    newPath[0] = 0;  // Start at current node (0)
    newPath[1] = target;
    findOptimalRoute(newPath, 2, optimizedPath, optimizedPathLength); // Corrected to 2 waypoints
    pathOptimized = true;
    
    // Print the optimized route for debugging
    Serial.println("\nStarting at node 0, facing from node 4");
    printRoute(optimizedPath, optimizedPathLength);

    // Initialize starting position correctly
    currentNode = 0;     // Robot starts at node 0
    previousNode = 4;    // Coming from direction of node 4
    nodesVisited = 0;    // Start with 0 nodes visited
    firstNode = true;    // Set first node flag

    while (begin == true) {
      distance = analogRead(distance_sensor);
      if (distance > 3000) {
        while (true) {
          distance = analogRead(distance_sensor);
          if (distance < 3000) {
            begin = false;
            delay(500);
            return;
          }
        }
      }
    }
}

void loop() {
  returnToLine();
  lineFollowPID();
  checkObstacle();

  if(currentNode == target) { 
    connectToWiFi();
    sendPostRequest(currentNode);  // Send current position as the arrived node
    String response = readResponse();
    String body = getResponseBody(response);
    int newTarget = body.toInt();
    client.stop();
    if (newTarget == 5) {
      newTarget = 6;
      parkingS = 1;
    }
    int newPath[21];
    newPath[0] = currentNode;  // Start from current position
    newPath[1] = newTarget;    // New target received
    findOptimalRoute(newPath, 2, optimizedPath, optimizedPathLength); // 2 waypoints
    target = newTarget;
    pathOptimized = true;
    printRoute(optimizedPath, optimizedPathLength);
    
    // Reset navigation variables for the new path
    nodesVisited = 0;
    firstNode = true;   // Treat the next node as the first in the new path
  }

  detectNode();
  delay(2);
}
