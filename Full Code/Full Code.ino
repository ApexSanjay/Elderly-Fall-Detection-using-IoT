#include <M5StickC.h>
#include <math.h>
#include <vector>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

//Define constants
const double THRESHOLD = 0.3;  //Fall detection threshold in m/s^2
const int WINDOW_SIZE = 50;    //window size for smoothing data

// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

//Define data structures
struct AccelerometerData {
  float x, y, z;
};

struct GyroscopeData {
  float x, y, z;
};

//Calculate magnitude of acceleration vector
float calculateMagnitude(AccelerometerData data) {
  return sqrt(pow(data.x, 2) + pow(data.y, 2) + pow(data.z, 2));
}

//Smooth Accelerometer data using a sliding window
std::vector<float> smoothData(std::vector<float> data) {
  std::vector<float> smoothedData;
  for (int i = 0; i <= data.size() - WINDOW_SIZE; i++) {
    float sum = 0;
    for (int j = i; j < i + WINDOW_SIZE; j++) {
      sum += data[j];
    }
    smoothedData.push_back(sum / WINDOW_SIZE);
  }
  return smoothedData;
}

// Function to detect falls based on accelerometer and gyroscope data
bool detectFall(const std::vector<AccelerometerData>& accelerometerData, const std::vector<GyroscopeData>& gyroscopeData) {
  // Calculate standard deviation of the z-axis accelerometer data
  float mean = 0.0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    mean += accelerometerData[i].z;
  }
  mean /= WINDOW_SIZE;

  float stdDev = 0.0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    stdDev += pow(accelerometerData[i].z - mean, 2);
  }
  stdDev = sqrt(stdDev / WINDOW_SIZE);

  // Print standard deviation to serial monitor
  Serial.print("Standard deviation: ");
  Serial.println(stdDev);

  // Detect fall if the standard deviation is above the threshold
  return stdDev > THRESHOLD;
}

void setCursorToLine(int lineNumber) {
  const int X_LOCATION = 5;
  const int Y_LOCATION = 0;
  const int LINE_HEIGHT = 10;
  M5.Lcd.setCursor(X_LOCATION, Y_LOCATION + ((lineNumber - 1) * LINE_HEIGHT), 2);
}

void printData(const std::vector<AccelerometerData>& accelerometerData, const std::vector<GyroscopeData>& gyroscopeData) {
  Serial.println("Accelerometer data:");
  for (const auto& data : accelerometerData) {
    Serial.printf("X=%.2f, Y=%.2f, Z=%.2f\n", data.x, data.y, data.z);
  }

  Serial.println("Gyroscope data:");
  for (const auto& data : gyroscopeData) {
    Serial.printf("X=%.2f, Y=%.2f, Z=%.2f\n", data.x, data.y, data.z);
  }
}

void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  // Create a message handler
  client.onMessage(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
  M5.Lcd.println("AWS IoT Connected!");
}

void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["sms"] = "+85265967075";
  doc["message"] = "Help! I've fallen and I can't get up!";
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  M5.Lcd.println(message);
}

void setup() {
  // Initialize the M5StickC object
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  Serial.begin(115200);
  M5.IMU.Init();
  connectAWS();
}

//Run repeatedly
void loop() {
  // Collect accelerometer and gyroscope data
  std::vector<AccelerometerData> accelerometerData;
  std::vector<GyroscopeData> gyroscopeData;
  for (int i = 0; i < 200; i++) {
    AccelerometerData accelData;
    GyroscopeData gyroData;
    M5.IMU.getAccelData((float*)&accelData.x, (float*)&accelData.y, (float*)&accelData.z);
    M5.IMU.getGyroData((float*)&gyroData.x, (float*)&gyroData.y, (float*)&gyroData.z);
    accelerometerData.push_back(accelData);
    gyroscopeData.push_back(gyroData);
    delay(10);
  }

  // Resize the data vectors to the actual number of data points collected
  accelerometerData.resize(200);
  gyroscopeData.resize(200);

  printData(accelerometerData, gyroscopeData);

  //Detect falls
  bool fallDetected = detectFall(accelerometerData, gyroscopeData);

  // Print result
  if (fallDetected) {
    setCursorToLine(3);
    M5.Lcd.printf("Fall detected!");
    Serial.println("Fall detected!");
    publishMessage();
  } else {
    setCursorToLine(3);
    M5.Lcd.printf("No fall detected.");
    Serial.println("No fall detected.");
  }

  // Wait before collecting more data
  delay(1000);
}