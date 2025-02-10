#include <Arduino.h>
#include <WiFi.h>

// Wi-Fi details
char ssid[] = "iot";
char password[] = "inflamedness65barra";

// Server details
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
bool firstNode = true;  // Flag to handle first node specially

// Path planning variables
int originalPath[] = {0, 4, 0, 3, 2, 0, 1, 3, 2, 4, 0};  // Initial waypoints
int optimizedPath[21];  // Will store the complete optimized path
int optimizedPathLength;
bool pathOptimized = false;

// Node connection layout with distances
const int nodeConnection[7][7] = {
  {0, 0, 0, 0, 2, 2, 0},
  {0, 0, 0, 0, 0, 2, 1},
  {0, 0, 0, 2, 0, 2, 0},
  {0, 0, 2, 0, 0, 0, 4},
  {2, 0, 0, 0, 0, 0, 4},
  {2, 2, 2, 0, 0, 0, 0},
  {0, 1, 0, 4, 4, 0, 0}
};

// Define turn directions for each valid node connection
const int nodeTurns[7][7][7] = {
    {{-1,-1,-1,-1,-1,-1,-1}, {1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,3,0,-1}, {-1,-1,-1,-1,0,3,-1}, {-1,-1,-1,-1,-1,-1,-1}},
    {{-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,3,0}, {-1,-1,-1,-1,-1,0,3}},
    {{-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,3,-1,0,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,0,-1,3,-1}, {-1,-1,-1,-1,-1,-1,-1}},
    {{-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,3,-1,-1,-1,0}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,0,-1,-1,-1,3}},
    {{3,-1,-1,-1,-1,-1,0}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {0,-1,-1,-1,-1,-1,3}},
    {{3,2,0,-1,-1,-1,-1}, {1,3,2,-1,-1,-1,0}, {0,1,3,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}},
    {{-1,-1,-1,-1,-1,-1,-1}, {-1,3,-1,1,2,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,2,-1,3,0,-1,-1}, {-1,1,-1,0,3,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}, {-1,-1,-1,-1,-1,-1,-1}}
};

// Wi-Fi functions
void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWi-Fi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

String readResponse() {
  String response = "";
  unsigned long timeout = millis() + 2000;
  while (client.connected() && millis() < timeout) {
    while (client.available()) {
      response += (char)client.read();
    }
  }
  return response;
}

int getStatusCode(String& response) {
  if (response.length() < 12) return 0;
  return response.substring(9, 12).toInt();
}

String getResponseBody(String& response) {
  int split = response.indexOf("\r\n\r\n");
  if (split == -1) return "";
  String body = response.substring(split + 4);
  body.trim();
  return body;
}

void sendPostRequest(int position) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to Wi-Fi...");
    connectToWiFi();
  }

  String postBody = "position=" + String(position);
  
  if (client.connect(server, port)) {
    client.println("POST /api/arrived/dggb1873 HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(postBody.length());
    client.println();
    client.println(postBody);
    delay(10); // Allow data to send
  } else {
    Serial.println("Server connection failed");
  }
}

// Path planning functions (unchanged from original code)
int minDistance(int dist[], bool visited[], int V) {
    int min = INT_MAX, min_index = -1;
    for (int v = 0; v < V; v++) {
        if (!visited[v] && dist[v] <= min) {
            min = dist[v];
            min_index = v;
        }
    }
    return min_index;
}

void getPath(int parent[], int dest, int path[], int& pathLength) {
    if (parent[dest] == -1) {
        path[0] = dest;
        pathLength = 1;
        return;
    }
    getPath(parent, parent[dest], path, pathLength);
    path[pathLength++] = dest;
}

void findShortestPath(int start, int end, int path[], int& pathLength) {
    const int V = 7;
    int dist[V], parent[V];
    bool visited[V];
    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        visited[i] = false;
        parent[i] = -1;
    }
    dist[start] = 0;
    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, visited, V);
        visited[u] = true;
        for (int v = 0; v < V; v++) {
            if (!visited[v] && nodeConnection[u][v] && dist[u] + nodeConnection[u][v] < dist[v]) {
                dist[v] = dist[u] + nodeConnection[u][v];
                parent[v] = u;
            }
        }
    }
    pathLength = 0;
    getPath(parent, end, path, pathLength);
}

int calculatePathDistance(int path[], int pathLength) {
    int total = 0;
    for (int i = 0; i < pathLength - 1; i++) total += nodeConnection[path[i]][path[i + 1]];
    return total;
}

void findOptimalRoute(int waypoints[], int numWaypoints, int finalPath[], int& finalPathLength) {
    finalPathLength = 0;
    int tempPath[7], tempLength;
    for (int i = 0; i < numWaypoints - 1; i++) {
        findShortestPath(waypoints[i], waypoints[i + 1], tempPath, tempLength);
        if (i < numWaypoints - 2) tempLength--;
        for (int j = 0; j < tempLength; j++) finalPath[finalPathLength++] = tempPath[j];
    }
}

void printRoute(int path[], int pathLength) {
    Serial.println("Optimized Route:");
    for (int i = 0; i < pathLength; i++) {
        Serial.print(path[i]);
        if (i < pathLength - 1) Serial.print(" -> ");
    }
    Serial.println("\nTotal Distance: " + String(calculatePathDistance(path, pathLength)));
}

// Motor control functions
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(motorL_PWM, constrain(leftSpeed, 0, MAX_SPEED));
  analogWrite(motorR_PWM, constrain(rightSpeed, 0, MAX_SPEED));
}

void stop_motor() {
  analogWrite(motorL_PWM, 0);
  analogWrite(motorR_PWM, 0);
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
  delay(700);
}

// Line following and navigation
void lineFollowPID() {
  digitalWrite(motorR_phase, LOW);
  digitalWrite(motorL_phase, LOW);

  int totalValue = 0, weightedSum = 0;
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

  setMotorSpeed(BASE_SPEED + correction, BASE_SPEED - correction);
}

void checkObstacle() {
  distance = analogRead(distance_sensor);
  if (distance > 2800) {
    stop_motor();
    turnAround();
  }
}

String getNextTurn(int current, int prev, int next) {
    int turn = nodeTurns[current][prev][next];
    switch(turn) {
        case 0: return "straight";
        case 1: return "left";
        case 2: return "right";
        case 3: return "around";
        default: return "straight";
    }
}

void handleNode() {
    if (!pathOptimized) {
        stop_motor();
        while(1);
        return;
    }

    Serial.print("Node detected! Current: ");
    Serial.print(currentNode);
    Serial.print(", Previous: ");
    Serial.print(previousNode);
    Serial.print(", Visited: ");
    Serial.println(nodesVisited);

    if (firstNode) {
        firstNode = false;
        currentNode = 0;
        previousNode = 4;
        int nextNode = optimizedPath[1];
        String turn = getNextTurn(currentNode, previousNode, nextNode);
        Serial.println("First turn: " + turn);
        if (turn == "left") turnLeft();
        else if (turn == "right") turnRight();
        else if (turn == "around") turnAround();
        previousNode = currentNode;
        currentNode = nextNode;
        nodesVisited = 1;
        return;
    }

    if (nodesVisited >= optimizedPathLength - 1) {
        stop_motor();
        sendPostRequest(currentNode);
        String response = readResponse();
        client.stop();
        int statusCode = getStatusCode(response);
        
        if (statusCode == 200) {
            String body = getResponseBody(response);
            if (body.equals("Finished")) {
                Serial.println("Mission complete!");
                while(1) { delay(1000); }
            } else {
                int nextTarget = body.toInt();
                int newPath[2] = {currentNode, nextTarget};
                findOptimalRoute(newPath, 2, optimizedPath, optimizedPathLength);
                nodesVisited = 0;
                Serial.println("New path received:");
                printRoute(optimizedPath, optimizedPathLength);
            }
        } else {
            Serial.println("Server error: " + String(statusCode));
        }
        return;
    }

    int nextNode = optimizedPath[nodesVisited + 1];
    String turn = getNextTurn(currentNode, previousNode, nextNode);
    Serial.print("Turn from " + String(currentNode) + " to " + String(nextNode) + ": " + turn);
    
    if (turn == "left") turnLeft();
    else if (turn == "right") turnRight();
    else if (turn == "around") turnAround();
    else {
        setMotorSpeed(BASE_SPEED, BASE_SPEED);
        delay(300);
    }
    
    previousNode = currentNode;
    currentNode = nextNode;
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
    delay(200);
  } else if (totalValue <= 15800) nodeDetected = false;
}

void setup() {
    for (int i = 0; i < 5; i++) pinMode(AnalogPin[i], INPUT);
    pinMode(motorR_PWM, OUTPUT);
    pinMode(motorR_phase, OUTPUT);
    pinMode(motorL_PWM, OUTPUT);
    pinMode(motorL_phase, OUTPUT);
    pinMode(distance_sensor, INPUT);
    
    Serial.begin(9600);
    connectToWiFi();

    int numWaypoints = sizeof(originalPath) / sizeof(originalPath[0]);
    if (originalPath[0] != 0) {
        int newPath[21] = {0};
        newPath[0] = 0;
        for (int i = 0; i < numWaypoints; i++) newPath[i + 1] = originalPath[i];
        findOptimalRoute(newPath, numWaypoints + 1, optimizedPath, optimizedPathLength);
    } else {
        findOptimalRoute(originalPath, numWaypoints, optimizedPath, optimizedPathLength);
    }
    pathOptimized = true;
    
    Serial.println("Starting at node 0");
    printRoute(optimizedPath, optimizedPathLength);
    currentNode = 0;
    previousNode = 4;
    nodesVisited = 0;
    firstNode = true;
}

void loop() {
  detectNode();
  lineFollowPID();
  checkObstacle();
  delay(2);
}