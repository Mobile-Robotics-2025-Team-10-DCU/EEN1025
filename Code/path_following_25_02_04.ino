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
int nodesVisited = 1;
int currentNode = 0;
int previousNode = 4;  // Robot starts from direction of node 4
bool firstNode = true;  // Flag to handle first node specially

// Path planning variables
int originalPath[] = {3, 1, 4, 0, 2, 0};  // Original waypoints
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
    if (!pathOptimized) {
        stop_motor();
        while(1); // Safety stop if path not optimized
        return;
    }

    if (firstNode) {
        // Special handling for first node
        firstNode = false;
        currentNode = 0;  // First node will be 0
        previousNode = 4; // Coming from direction of node 4
        nodesVisited = 0;
        
        // If 0 is not our first waypoint, we need to turn appropriately
        if (optimizedPath[0] != 0) {
            String turn = getNextTurn(0, 4, optimizedPath[0]);
            if (turn == "left") {
                turnLeft();
            } else if (turn == "right") {
                turnRight();
            } else if (turn == "around") {
                turnAround();
            }
        }
        return;
    }

    if (nodesVisited >= optimizedPathLength - 1) {
        stop_motor();
        while(1); // End of path
        return;
    }

    currentNode = optimizedPath[nodesVisited];
    int nextNode = optimizedPath[nodesVisited + 1];
    
    // Get the turn direction based on current position and next node
    String turn = getNextTurn(currentNode, previousNode, nextNode);
    
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
        delay(300);
    }
    
    previousNode = currentNode;
    nodesVisited++;
    
    // Print debug information
    Serial.print("Visited node: ");
    Serial.print(currentNode);
    Serial.print(" Next node: ");
    Serial.println(nextNode);
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
    
    Serial.begin(9600);
    
    // Calculate the optimized path at startup
    int numWaypoints = sizeof(originalPath) / sizeof(originalPath[0]);
    
    // If first waypoint is not 0, prepend 0 to the path
    if (originalPath[0] != 0) {
        int newPath[21];  // Temporary array for the new path
        newPath[0] = 0;  // Start with node 0
        for (int i = 0; i < numWaypoints; i++) {
            newPath[i + 1] = originalPath[i];
        }
        findOptimalRoute(newPath, numWaypoints + 1, optimizedPath, optimizedPathLength);
    } else {
        findOptimalRoute(originalPath, numWaypoints, optimizedPath, optimizedPathLength);
    }
    pathOptimized = true;
    
    // Print the optimized route for debugging
    Serial.println("Starting between nodes 0 and 4, facing 0");
    printRoute(optimizedPath, optimizedPathLength);
}

void loop() {
  detectNode();
  lineFollowPID();
  checkObstacle();
  delay(2);
}