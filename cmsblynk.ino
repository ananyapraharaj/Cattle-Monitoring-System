#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "MAX30100_PulseOximeter.h"
#define BLYNK_TEMPLATE_ID "TMPL3Ox-2WLKx"
#define BLYNK_TEMPLATE_NAME "CattleWatch"
#define BLYNK_DEVICE_NAME "CattleWatchr"
#define BLYNK_AUTH_TOKEN "Q6RENhZI8Lb3jFtz-Uoww8p3UXRrRcX6"
#define WIFI_SSID "Ananya's S23"
#define WIFI_PASS "iworshipananya"

#define DHTPIN 5
#define DHTTYPE DHT11
#define GPS_RX 2
#define GPS_TX 4
#define PUSH_BUTTON_PIN 12

DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
PulseOximeter pox;

BlynkTimer timer;
float latitude, longitude;

void connectWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi!");
}

void readDHTSensor() {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
        Blynk.virtualWrite(V1, t);
        Blynk.virtualWrite(V2, h);
        Serial.print("Temp: "); Serial.print(t);
        Serial.print(" C, Humidity: "); Serial.println(h);
    }
}

void readPulseOximeter() {
    pox.update();
    float bpm = pox.getHeartRate();
    float spo2 = pox.getSpO2();
    Blynk.virtualWrite(V3, bpm);
    Blynk.virtualWrite(V4, spo2);
    Serial.print("BPM: "); Serial.print(bpm);
    Serial.print(", SpO2: "); Serial.println(spo2);
}

void readGPSData() {
    while (GPSSerial.available() > 0) {
        gps.encode(GPSSerial.read());
    }
    if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Blynk.virtualWrite(V5, latitude);
        Blynk.virtualWrite(V6, longitude);
        Serial.print("Lat: "); Serial.print(latitude, 6);
        Serial.print(", Long: "); Serial.println(longitude, 6);
    }
}

void sendGPSLocation() {
    if (gps.location.isValid()) {
        Serial.println("Button Pressed: Sending GPS Location");
        Blynk.virtualWrite(V7, String(latitude, 6) + "," + String(longitude, 6));
    }
}

void setup() {
    Serial.begin(115200);
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    dht.begin();
    pinMode(PUSH_BUTTON_PIN, INPUT);
    
    connectWiFi();
    Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
    
    pox.begin();
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    
    timer.setInterval(2000L, readDHTSensor);
    timer.setInterval(1000L, readPulseOximeter);
    timer.setInterval(3000L, readGPSData);
}

void loop() {
    Blynk.run();
    timer.run();
    
    if (digitalRead(PUSH_BUTTON_PIN) == HIGH) {
        sendGPSLocation();
    }
}