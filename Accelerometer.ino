#define BLYNK_TEMPLATE_ID "TMPL3XkmVspUL"
#define BLYNK_TEMPLATE_NAME "Accelerometer"
#define BLYNK_AUTH_TOKEN "QCfmmCfmKM-tBQjfNlC0LewgR8kF6AAl"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <MPU6050.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "";  // Enter your WiFi name
char pass[] = "";  // Enter your WiFi password

BlynkTimer timer;
MPU6050 mpu;

void sendSensor()
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float accelX = ax / 16384.0;  // Convert to g-force
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    
    float gyroX = gx / 131.0; // Convert to deg/sec
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    Blynk.virtualWrite(V0, accelX);
    Blynk.virtualWrite(V1, accelY);
    Blynk.virtualWrite(V2, accelZ);
    Blynk.virtualWrite(V3, gyroX);
    Blynk.virtualWrite(V4, gyroY);
    Blynk.virtualWrite(V5, gyroZ);

    Serial.print("Accel (g): X="); Serial.print(accelX);
    Serial.print(" Y="); Serial.print(accelY);
    Serial.print(" Z="); Serial.println(accelZ);
    
    Serial.print("Gyro (deg/sec): X="); Serial.print(gyroX);
    Serial.print(" Y="); Serial.print(gyroY);
    Serial.print(" Z="); Serial.println(gyroZ);
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    
    Blynk.begin(auth, ssid, pass);
    timer.setInterval(100L, sendSensor);
}

void loop()
{
    Blynk.run();
    timer.run();
}
