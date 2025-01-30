#include <WiFi.h>

// Wi-Fi details
char ssid[] = "iot";
char password[] = "inflamedness65barra";

// Server details
char server[] = "3.250.38.184";
int port = 8000;

// WiFiClient object for communication
WiFiClient client;

// Function to connect to Wi-Fi
void connectToWiFi() {
  Serial.print("Connecting to network: ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Function to send a POST request with position
void sendPostRequest(int position) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected. Reconnecting...");
    connectToWiFi();
  }

  String postBody = "position=" + String(position);
  
  Serial.print("Connecting to server: ");
  Serial.print(server);
  Serial.print(":");
  Serial.println(port);

  if (client.connect(server, port)) {
    Serial.println("Connected to server");
    
    client.println("POST /api/arrived/dggb1873 HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    client.println(postBody.length());
    client.println();
    client.println(postBody);

    Serial.println("Request sent");
  } else {
    Serial.println("Connection to server failed");
  }
}

// Function to read the HTTP response from the server
String readResponse() {
  String response = "";
  unsigned long timeout = millis() + 2000;  // 5-second timeout
  while (client.connected() && millis() < timeout) {
    while (client.available()) {
      response += (char)client.read();
    }
  }
  if (response == "") {
    Serial.println("No response or timeout occurred");
  }
  return response;
}

// Function to get the status code from the response
int getStatusCode(String& response) {
  if (response.length() < 12) return 0;  // Ensure response has sufficient length
  String code = response.substring(9, 12);
  return code.toInt();
}

// Function to extract the response body from the HTTP response
String getResponseBody(String& response) {
  int split = response.indexOf("\r\n\r\n");
  if (split == -1) {
    return "";  // Invalid response
  }
  String body = response.substring(split + 4);
  body.trim();
  return body;
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  connectToWiFi();
}

void loop() {
  int position = 0;  // Start at node 0
  int destination = -1;  // Initialize the destination

  while (true) {  // Keep looping until we confirm reaching node 5
    sendPostRequest(position);  // Send the current position to the server
    
    // Read and parse the server response
    String response = readResponse();
    Serial.println("Full Response:");
    Serial.println(response);
    
    int statusCode = getStatusCode(response);
    
    if (statusCode == 200) {  // If the request was successful
      String body = getResponseBody(response);
      Serial.print("Server response body: ");
      Serial.println(body);  // Log the body to debug

      if (body.equals("Finished")) {
        Serial.println("Final destination received! Confirming arrival at node 5...");

        // **Ensure the ESP explicitly confirms reaching node 5**
        sendPostRequest(5);
        Serial.println("Final confirmation sent to server.");
        
        break; // Now we can exit the loop
      } else {
        destination = body.toInt();  // Get the next destination
        Serial.print("Moving to destination: ");
        Serial.println(destination);
        position = destination;  // Update the current position
      }
    } else {
      Serial.print("HTTP Error: ");
      Serial.println(statusCode);
    }
    
    client.stop();  // Disconnect from the server
    delay(1000);  // Delay to simulate movement or avoid spamming the server
  }

  Serial.println("Traversal complete. Loop stopping.");
  while (true) {}  // Stop the loop once traversal is complete
}
