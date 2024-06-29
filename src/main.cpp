#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <WiFi.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Preferences.h>
#include <time.h>

#define RESET_H 19
#define RESET_M 00

Preferences preferences;
const char *key = "stepCount";

// for gps
static const int RXPin = 27, TXPin = 14;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
double lat = 0;
double lng = 0;

// for heart rate sensor
MAX30105 particleSensor;
const byte RATE_SIZE = 4; 
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

// for step counter
Adafruit_MPU6050 mpu;
const int step_threshold = 5; // Adjust this threshold based on calibration
int step_count = 0;
static float last_accel_magnitude = 0;

// for firebase

//  Firebase API_KEY
#define API_KEY "AIzaSyD7F8qnmzZE8vp7eQibAIsFojcYYTfWjwk"
// Firebase RTDB URL
#define DATABASE_URL "pet-plus-pro-00-default-rtdb.firebaseio.com"
// Authentication via email and password for additional security
#define USER_EMAIL "admin@petproplus.com"
#define USER_PASSWORD "123456"
/* Define the WiFi credentials */
#define WIFI_SSID "Redmi Note 11"
#define WIFI_PASSWORD "12345678"
String path = "Device";
// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// declare functions
void loop2(void *pvParameters);
void getHeatRate();
void getStepCount();

void setup()
{
  Serial.begin(115200);

  // Initialize Preferences with the "my-app" namespace
  preferences.begin("my-app", false);
  // Read the stored step count (default to 0 if the key doesn't exist)
  step_count = preferences.getInt(key, 0);

  // GPS
  ss.begin(GPSBaud);

  // MPU6050
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  //Heart rate sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST);
  particleSensor.setup();                    // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED


  // WIFI
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  

  // Firebase
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
  fbdo.keepAlive(5, 5, 1);

  // Initialize NTP
  configTime(19800, 0, "pool.ntp.org");


  // Pin to the task for Core 0 in esp32
  xTaskCreatePinnedToCore(
      loop2,   // Function to implement the task
      "loop2", // Name of the task
      20000,   // Stack size in bytes
      NULL,    // Task input parameter
      1,       // Priority of the task
      NULL,    // Task handle.
      0        // Core where the task should run
  );
}

void loop()
{

  getHeatRate();
  getStepCount();

}

void loop2(void *pvParameters)
{

  while (1)
  {
    while (ss.available() > 0)
    {
      gps.encode(ss.read());
      lat = gps.location.lat();
      lng = gps.location.lng();
    }

    //create a stucture for store time information
    struct tm timeInfo;

    //get the time information
    getLocalTime(&timeInfo);

    //create a json object to update firebase RTDB
    FirebaseJson json;

    //add data to json object
    json.add("heart_rate", beatAvg);
    json.add("steps", step_count);
    json.add("longitude", lng);
    json.add("latitude", lat);

    //update the json object to the firebase 
    Firebase.RTDB.updateNodeSilentAsync(&fbdo, path, &json);

    //update the daily step count with the information of the date
    if (timeInfo.tm_hour == RESET_H && timeInfo.tm_min == RESET_M && timeInfo.tm_sec == 0)
    {
      json.add("history/" + String(timeInfo.tm_year) + "-"+ String(timeInfo.tm_mon) + "-"+ String(timeInfo.tm_mday) + "-", step_count);
      Firebase.RTDB.updateNodeSilentAsync(&fbdo, path, &json);
      
    }
    //Reset the step count
    if (timeInfo.tm_hour == RESET_H && timeInfo.tm_min == RESET_M + 1 && timeInfo.tm_sec == 0){
      step_count = 0;
      preferences.putInt(key, 0);
    }
    
  }
}

//get the heart rate
void getHeatRate()
{
  long irValue = particleSensor.getIR();
  if (irValue < 50000)
  {
    beatAvg = 0;
  }
  else
  {
    if (checkForBeat(irValue) == true)
    {
      // We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= RATE_SIZE;                    // Wrap variable

        // Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  }
}

//get the step count
void getStepCount()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //get the magnitude of the accelaration vector
  float current_accel_magnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                       a.acceleration.y * a.acceleration.y +
                                       a.acceleration.z * a.acceleration.z);


  //check for a peek of a accelaration vector                                     
  if (current_accel_magnitude - last_accel_magnitude > step_threshold)
  {
    step_count++;
    preferences.putInt(key, step_count);
  }

  last_accel_magnitude = current_accel_magnitude;
}


