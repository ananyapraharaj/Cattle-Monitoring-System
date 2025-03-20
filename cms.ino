#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "MAX30100_PulseOximeter.h"
#include <UniversalTelegramBot.h>

#define IO_USERNAME "############"
#define IO_KEY "#####################################"

#define WIFI_SSID "#######"
#define WIFI_PASS "###########"

#define DHTPIN 5
#define DHTTYPE DHT11

#define GPS_RX 2
#define GPS_TX 4
#define I2C_SDA 21
#define I2C_SCL 22

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, "io.adafruit.com", 1883, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Publish GPSLocation = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/gps/csv");
Adafruit_MQTT_Publish bpm_pub = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/heartbeat");
Adafruit_MQTT_Publish spo2_pub = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/spO2");
Adafruit_MQTT_Subscribe sw_sub = Adafruit_MQTT_Subscribe(&mqtt, IO_USERNAME "/feeds/switch");

DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Set up a software serial port for the GPS module
PulseOximeter pox;
float speed_mph = 0;
float altitude = 0;
float lati;   // Storing the Latitude
float longi;  // Storing the Longitude

char gpsdata[120];
TaskHandle_t poxReadTaskHld = NULL;
TaskHandle_t mqttPubTaskHld = NULL;

// Telegram BOT Token (Get from Botfather)
#define BOT_TOKEN "6585668503:AAHHHyBfLI4V0Hfjx4kmKw9acgiGKJH5bdA"

const unsigned long BOT_MTBS = 1000;        // mean time between scan messages
const unsigned long HELLO_INTERVAL = 5000;  // time interval for sending "Hello" (in milliseconds)

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);
unsigned long bot_lasttime;    // last time messages' scan has been done
unsigned long hello_lasttime;  // last time "Hello" message was sent

#define PUSH_BUTTON_PIN 12
int prevButtonState = LOW;

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
}

void connectAdafruit() {
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Connecting to Adafruit MQTT... ");
    delay(5000);
  }
  Serial.println("Connected to Adafruit MQTT!");
}

void onBeatDetected() {
  Serial.println("Beat!");
}

void stopReadPOX() {
  pox.shutdown();
}

void startReadPOX() {
  pox.resume();
}

void poxReadTask(void *param) {
  while (1) {
    pox.update();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  poxReadTaskHld = NULL;
  vTaskDelete(NULL);
}

void mqttPubTask(void *param) {
  uint8_t sec_count = 0;
  while (1) {
    float bpm_dt = pox.getHeartRate();
    float spo2_dt = pox.getSpO2();

    if (sec_count >= 5) {
      bpm_pub.publish(bpm_dt);
      spo2_pub.publish(spo2_dt);
      sec_count = 0;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    sec_count++;
  }
  mqttPubTaskHld = NULL;
  vTaskDelete(NULL);
}

String currentChatId;  // Declare chat_id as a global variable

void handleNewMessages(int numNewMessages) {
  for (int i = 0; i < numNewMessages; i++) {
    currentChatId = bot.messages[i].chat_id;
    String text = bot.messages[i].text;
    String from_name = bot.messages[i].from_name;

    if (from_name == "") from_name = "Guest";

    if (text == "/hello") {
      sendHello(currentChatId);
    } else if (text == "/help") {
      sendGPSLocation(currentChatId);
    }

    if (text == "/start") {
      sendWelcome(currentChatId, from_name);
    }
  }
}

void sendHello(String chat_id) {
  bot.sendMessage(chat_id, "Hello from your bot!", "");
}


void sendGPSLocation(String chat_id) {
  if (gps.location.isValid()) {
    String locationLink = "https://maps.google.com/?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    bot.sendMessage(chat_id, "Here is your current location: " + locationLink, "");
  } else {
    bot.sendMessage(chat_id, "Unable to determine current location.", "");
  }
}

void sendWelcome(String chat_id, String from_name) {
  String welcome = "Welcome to Universal Arduino Telegram Bot library, " + from_name + ".\n";
  welcome += "This is a Hello Bot example.\n\n";
  welcome += "/hello : to receive a greeting\n";
  welcome += "/help : to get the current GPS coordinates link\n";
  bot.sendMessage(chat_id, welcome, "Markdown");
}



void setup() {
  pinMode(PUSH_BUTTON_PIN, INPUT);
  mqtt.subscribe(&sw_sub);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);  // Add root certificate for api.telegram.org
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("PASHU- THE PET HEALTH BAND");
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Initializing Band......");
  display.display();
  delay(2000);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Finding server.....");
  display.display();
  delay(2000);

  connectWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connected to Wi-Fi!");
    display.display();
    delay(2000);
  } else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Not connected to WiFi");
    display.display();
    while (true)
      ;
  }

  connectAdafruit();

  if (mqtt.connected()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to Adafruit MQTT...");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connected To server");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Band Connected to server");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Check your Health band dashboard");
    display.display();
    delay(2000);
  }


  Serial.print("Retrieving time: ");
  configTime(0, 0, "pool.ntp.org");  // get UTC time via NTP
  time_t now = time(nullptr);
  while (now < 24 * 3600) {
    Serial.print(".");
    delay(100);
    now = time(nullptr);
  }
  Serial.println(now);
  hello_lasttime = millis();

  pox.begin();
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
  stopReadPOX();

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  dht.begin();
}

void loop() {
  int buttonState = digitalRead(PUSH_BUTTON_PIN);

  if (buttonState == HIGH) {
    Serial.println("Button Pressed");
    sendGPSLocation(currentChatId);
    Serial.println("Message sent");
  }

  prevButtonState = buttonState;

  if (millis() - hello_lasttime > HELLO_INTERVAL) {
    sendHello("your_chat_id");
    hello_lasttime = millis();
  }

  if (millis() - bot_lasttime > BOT_MTBS) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
  }

  // Delay before reading DHT11 data
  delay(1000);  // Add a delay before reading DHT11 data

  // Read data from DHT11
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.println(" %");

    Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/temperature");
    temperature.publish(String(t).c_str());

    Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/humidity");
    humidity.publish(String(h).c_str());
  }

  getCoordinates();

  Serial.print("Lati = ");
  Serial.print(lati, 6);
  Serial.print("\tLongi = ");
  Serial.println(longi, 6);

  if (!GPSLocation.publish(gpsdata)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("Sent!"));
  }

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &sw_sub) {
      if (!strcmp((char *)sw_sub.lastread, "ON")) {
        // Start the POX and MQTT tasks as before
        startReadPOX();
        BaseType_t xReturned;

        if (poxReadTaskHld == NULL) {
          xReturned = xTaskCreate(poxReadTask, "pox_read", 1024 * 3, NULL, 2, &poxReadTaskHld);
        }

        if (mqttPubTaskHld == NULL) {
          xReturned = xTaskCreate(mqttPubTask, "mqttPub", 1024 * 3, NULL, 2, &mqttPubTaskHld);
        }
      } else {
        // Stop the POX and MQTT tasks as before
        if (poxReadTaskHld != NULL) {
          vTaskDelete(poxReadTaskHld);
          poxReadTaskHld = NULL;
        }

        if (mqttPubTaskHld != NULL) {
          vTaskDelete(mqttPubTaskHld);
          mqttPubTaskHld = NULL;
        }

        stopReadPOX();
      }
    }
  }

  mqtt.processPackets(5000);
  delay(5000);
}

void getCoordinates() {
  readGPSData();
  char *p = gpsdata;

  dtostrf(speed_mph, 2, 6, p);
  p += strlen(p);
  p[0] = ',';
  p++;

  dtostrf(lati, 2, 6, p);
  p += strlen(p);
  p[0] = ',';
  p++;

  dtostrf(longi, 3, 6, p);
  p += strlen(p);
  p[0] = ',';
  p++;

  dtostrf(altitude, 2, 6, p);
  p += strlen(p);

  p[0] = 0;
}

void readGPSData() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  if (gps.location.isValid()) {
    lati = gps.location.lat();
    longi = gps.location.lng();
    Serial.print("Lati: ");
    Serial.print(lati, 6);
    Serial.print("\tLongi: ");
    Serial.println(longi, 6);
  }
  waitGPS(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println("Waiting for data...");
}

static void waitGPS(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (GPSSerial.available())
      gps.encode(GPSSerial.read());
  } while (millis() - start < ms);
}