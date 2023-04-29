#include <M5StickC.h>
#include <math.h>
#include <vector>

//Define constants
const double THRESHOLD = 0.3;  //Fall detection threshold in m/s^2
const int WINDOW_SIZE = 50;    //window size for smoothing data

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

void setup() {
  // Initialize the M5StickC object
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  Serial.begin(115200);
  M5.IMU.Init();
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
  } else {
    setCursorToLine(3);
    M5.Lcd.printf("No fall detected.");
    Serial.println("No fall detected.");
  }

  // Wait before collecting more data
  delay(1000);
}