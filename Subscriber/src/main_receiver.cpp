#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi Credentials (same as the AP)
const char* ssid = "ESP32-Access-Point-Steven"; 
const char* password = "12345678";  

// ESP32 Server Address (Server must be running on AP)
const char* serverNameAng = "http://192.168.4.1/angles";  

String receivedAngles;
unsigned long previousMillis = 0;
const long interval = 200;  // Fetch data every second

// Function to perform an HTTP GET request
String httpGETRequest(const char* serverName) {
  HTTPClient http;
  http.begin(serverName);
  
  int httpResponseCode = http.GET();
  String payload = "--"; 
  
  if (httpResponseCode > 0) {
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();  // Free resources
  return payload;
}

void setup() {
  Serial.begin(9600);
  Serial.println("\nConnecting to WiFi...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected! IP Address: " + WiFi.localIP().toString());
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    if (WiFi.status() == WL_CONNECTED) { 
       receivedAngles = httpGETRequest(serverNameAng);
       
       Serial.println("Received JSON: " + receivedAngles);

       // Extract values manually (String Parsing)
       int rollDCM_start = receivedAngles.indexOf("\"rollDCM\":") + 10;
       int pitchDCM_start = receivedAngles.indexOf("\"pitchDCM\":") + 11;
       int yawDCM_start = receivedAngles.indexOf("\"yawDCM\":") + 9;
       int rollQ_start = receivedAngles.indexOf("\"rollQ\":") + 8;
       int pitchQ_start = receivedAngles.indexOf("\"pitchQ\":") + 9;
       int yawQ_start = receivedAngles.indexOf("\"yawQ\":") + 7;

       float rollDCM = receivedAngles.substring(rollDCM_start, receivedAngles.indexOf(",", rollDCM_start)).toFloat();
       float pitchDCM = receivedAngles.substring(pitchDCM_start, receivedAngles.indexOf(",", pitchDCM_start)).toFloat();
       float yawDCM = receivedAngles.substring(yawDCM_start, receivedAngles.indexOf(",", yawDCM_start)).toFloat();
       float rollQ = receivedAngles.substring(rollQ_start, receivedAngles.indexOf(",", rollQ_start)).toFloat();
       float pitchQ = receivedAngles.substring(pitchQ_start, receivedAngles.indexOf(",", pitchQ_start)).toFloat();
       float yawQ = receivedAngles.substring(yawQ_start, receivedAngles.indexOf("}", yawQ_start)).toFloat();

       // Display Extracted Values
       Serial.print("Roll DCM: "); Serial.print(rollDCM);
       Serial.print(" | Pitch DCM: "); Serial.print(pitchDCM);
       Serial.print(" | Yaw DCM: "); Serial.println(yawDCM);
       
       Serial.print("Roll Q: "); Serial.print(rollQ);
       Serial.print(" | Pitch Q: "); Serial.print(pitchQ);
       Serial.print(" | Yaw Q: "); Serial.println(yawQ);
       Serial.println("----------------------");
       
    } else {
       Serial.println("WiFi not connected. Retrying...");
    }
    previousMillis = currentMillis;
  }
}
