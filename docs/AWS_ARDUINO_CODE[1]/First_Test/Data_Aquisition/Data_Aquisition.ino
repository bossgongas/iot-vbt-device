#include "Arduino.h"
#include "secrets.h"
#include <WiFiClientSecure.h>
#include "WiFi.h"
#include "MPU9250.h"
#include <MQTTClient.h>
#include <ArduinoJson.h>

// Define the touch pin
const int touchPin = T0; // GPIO4
unsigned long lastTouchTime = 0; // To debounce touch input
const unsigned long debounceDelay = 100; // Debounce delay in milliseconds

#define AWS_IOT_PUBLISH_TOPIC "ESP_32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "ESP_32/sub"

unsigned long lastSendingTime = 0;
const unsigned long sendMessageEveryMillis =500;  // Send every 5 seconds
const float velocityThreshold = 2.0; // Threshold to detect the start of an exercise
int exerciseCounter = 0; // Exercise identifier
bool exerciseOngoing = false; // Exercise state

MPU9250 IMU(Wire, 0x68); // Initialize the MPU9250 on I2C
WiFiClientSecure wifi_client = WiFiClientSecure();
MQTTClient client = MQTTClient(512); // Increase buffer size if needed

float accelBias[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0};

void messageHandler(String &topic, String &payload) {
    Serial.println("Incoming: [" + topic + "]");
    Serial.println("---------------------------");

    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    serializeJsonPretty(doc, Serial);
    Serial.println();
    Serial.println("---------------------------");
}

bool connectAWS() {
    // Configure WiFiClientSecure for the AWS IoT device certificate
    wifi_client.setCACert(AWS_CERT_CA);
    wifi_client.setCertificate(AWS_CERT_CRT);
    wifi_client.setPrivateKey(AWS_CERT_PRIVATE);

    // Connect to the MQTT broker on the AWS endpoint
    client.begin(AWS_IOT_ENDPOINT, 8883, wifi_client);

    // Create a message handler
    client.onMessage(messageHandler);

    Serial.println("Connecting to AWS IoT");

    while (!client.connect(THINGNAME)) {
        Serial.print(".");
        delay(100);
    }

    if (!client.connected()) {
        Serial.println("AWS IoT Timeout!");
        return false;
    }

    // Subscribe to a topic
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

    return true;
}

void calibrateSensor() {
    Serial.println("Calibrating sensor...");
    float accelSum[3] = {0, 0, 0};
    float gyroSum[3] = {0, 0, 0};
    int samples = 1000;

    for (int i = 0; i < samples; i++) {
        IMU.readSensor();
        accelSum[0] += IMU.getAccelX_mss();
        accelSum[1] += IMU.getAccelY_mss();
        accelSum[2] += IMU.getAccelZ_mss();
        gyroSum[0] += IMU.getGyroX_rads();
        gyroSum[1] += IMU.getGyroY_rads();
        gyroSum[2] += IMU.getGyroZ_rads();
        delay(10);
    }

    accelBias[0] = accelSum[0] / samples;
    accelBias[1] = accelSum[1] / samples;
    accelBias[2] = accelSum[2] / samples;
    gyroBias[0] = gyroSum[0] / samples;
    gyroBias[1] = gyroSum[1] / samples;
    gyroBias[2] = gyroSum[2] / samples;

    Serial.println("Calibration complete.");
}

bool publishMessage() {
    IMU.readSensor();

    // Retrieve calibrated accelerometer data
    float accelX = IMU.getAccelX_mss() - accelBias[0];
    float accelY = IMU.getAccelY_mss() - accelBias[1];
    float accelZ = IMU.getAccelZ_mss() - accelBias[2];

    // Retrieve calibrated gyroscope data
    float gyroX = IMU.getGyroX_rads() - gyroBias[0];
    float gyroY = IMU.getGyroY_rads() - gyroBias[1];
    float gyroZ = IMU.getGyroZ_rads() - gyroBias[2];

     // Increment the exercise counter
    Serial.println("Exercise ongoing!");

    // Print accelerometer and gyroscope values to the Serial Monitor
    Serial.print("Sending Accel X: ");
    Serial.print(accelX);
    Serial.print(" m/s^2, Accel Y: ");
    Serial.print(accelY);
    Serial.print(" m/s^2, Accel Z: ");
    Serial.println(accelZ);
    Serial.println(" m/s^2");

    Serial.print("Sending Gyro X: ");
    Serial.print(gyroX);
    Serial.print(" rad/s, Gyro Y: ");
    Serial.print(gyroY);
    Serial.print(" rad/s, Gyro Z: ");
    Serial.println(gyroZ);
    Serial.println(" rad/s");

    // Create JSON document
    DynamicJsonDocument doc(512);

    doc["ExerciseID"] = exerciseCounter;
    doc["AccX"] = accelX;
    doc["AccY"] = accelY;
    doc["AccZ"] = accelZ;
    doc["GyroX"] = gyroX;
    doc["GyroY"] = gyroY;
    doc["GyroZ"] = gyroZ;

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer); // serialize the JSON document to a buffer

    Serial.print("Message on ");
    Serial.print(AWS_IOT_PUBLISH_TOPIC);
    Serial.print(" topic... ");

    // Publish message to AWS_IOT_PUBLISH_TOPIC
    if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer)) {
        Serial.println("Published!");
        return true;
    } else {
        Serial.println("Error!");
        return false;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid, password);

    Serial.print("Connecting to ");
    Serial.print(ssid); Serial.println("");

    while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
        delay(1000);
        Serial.print('.');
    }

    Serial.println('\n');
    Serial.println("Connection established!");
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP());

    connectAWS();

    int status = 0; 

    status = IMU.begin();
    if (status < 0) {
        Serial.println("MPU9250 connection failed.");
        while (1);
    } else {
        Serial.println("MPU9250 is connected successfully.");
              
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
   
        IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);

        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_5HZ);
          }

    calibrateSensor();
    Serial.println("Touch Sensor Test");
}

void loop() {
    client.loop();
    int touchValue = touchRead(touchPin);

    // Check for touch input with debounce
    if (touchValue < 40 && millis() - lastTouchTime > debounceDelay) {
        lastTouchTime = millis();
        exerciseOngoing = !exerciseOngoing; // Toggle exercise state

        if (exerciseOngoing) {
            Serial.println("Exercise started!");
        } else {
            exerciseCounter++;
            Serial.println("Exercise stopped!");
        }
    }

    if (exerciseOngoing && millis() - lastSendingTime > sendMessageEveryMillis) {
        publishMessage();
        lastSendingTime = millis();
    }

    delay(50);
}
