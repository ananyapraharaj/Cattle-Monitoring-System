#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME "CattlePulse"
#define BLYNK_AUTH_TOKEN ""

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

const char* ssid = "Ananya's S23";  // Replace with your Wi-Fi SSID
const char* password = "";  // Replace with your Wi-Fi password
const int pulsePin = 36; // VP = GPIO 36


int heartRate = 0;  // Variable to store heart rate

BlynkTimer timer;  // Timer for periodic updates to Blynk

// This function sends heart rate to Blynk every second
void sendHeartRateToBlynk() {
  delay(10);
  int sensorValue = analogRead(pulsePin);
  
  // Print raw ADC values for debugging
  Serial.print("Raw ADC Value: ");
  Serial.println(sensorValue);
  
  // Map ADC value to BPM range
  sensorValue = constrain(sensorValue, 250, 600);  
  heartRate = map(sensorValue, 0, 250, 50, 150);  

  Serial.print("Heart Rate (BPM): ");
  Serial.println(heartRate);

  Blynk.virtualWrite(V1, heartRate); // Send to Blynk
}


void setup() {
  Serial.begin(9600);  // Start serial communication

  WiFi.begin(ssid, password);  // Connect to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {  // Wait until connected
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);  // Initialize Blynk with auth token and Wi-Fi credentials

  timer.setInterval(1000L, sendHeartRateToBlynk);  // Set a timer to send heart rate data every 1000 ms (1 second)
}

void loop() {
  Blynk.run();  // Run Blynk's main loop
  timer.run();  // Run the Blynk timer
}
