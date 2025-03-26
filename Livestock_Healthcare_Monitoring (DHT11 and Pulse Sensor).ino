#define BLYNK_TEMPLATE_ID "TMPL3n0TwXHgw"
#define BLYNK_TEMPLATE_NAME "Livestock Healthcare Monitoring"
#define BLYNK_AUTH_TOKEN "KdH_KqsPwkIIedbM-iQxaucVzNyAgIYm"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

// WiFi credentials
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Siddhartha";  // WiFi Name
char pass[] = "98765432";    // WiFi Password

BlynkTimer timer;

// ********** DHT11 Sensor Setup **********
#define DHTPIN 27         // DHT11 Data Pin
#define DHTTYPE DHT11     // Define Sensor Type
DHT dht(DHTPIN, DHTTYPE);

// ********** Pulse Sensor Setup **********
const int pulsePin = 32; // Pulse Sensor Pin
const int ledPin = 33;   // LED Indicator
int pulseValue;          // Raw Pulse Sensor Value
int bpm;                 // Beats Per Minute

// ********** Function to Read DHT11 Sensor **********
void sendDHTData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print("°C  |  Humidity: ");
  Serial.print(h);
  Serial.println("%");

  // Send Data to Blynk
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
}

// ********** Function to Read Pulse Sensor **********
void sendPulseData() {
  pulseValue = analogRead(pulsePin);

  // If pulse is detected (threshold > 600), calculate BPM
  if (pulseValue > 600) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    
    bpm = 60000 / pulseValue;  // Calculate BPM

    Serial.print("Heart Rate: ");
    Serial.print(bpm);
    Serial.println(" BPM");

    // Send BPM data to Blynk
    Blynk.virtualWrite(V2, bpm);
  }
}

void setup() {
  Serial.begin(115200);

  // Start DHT11 Sensor
  dht.begin();

  // Connect to WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to Blynk
  Blynk.begin(auth, ssid, pass);
  while (!Blynk.connected()) {
    Serial.println("Connecting to Blynk...");
    delay(1000);
  }
  Serial.println("Connected to Blynk");

  // Pulse Sensor Setup
  pinMode(pulsePin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Set Timers
  timer.setInterval(1000L, sendDHTData);  // Update DHT11 every 1 second
  timer.setInterval(500L, sendPulseData); // Update BPM every 500ms
}

void loop() {
  Blynk.run();
  timer.run();
}
