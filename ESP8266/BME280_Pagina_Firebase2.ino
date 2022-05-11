/*
  Rui Santos
  Complete project details at our blog: https://RandomNerdTutorials.com/esp8266-data-logging-firebase-realtime-database/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#endif

#define SEALEVELPRESSURE_HPA (1013.25)

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "Mastero Wifi"
#define WIFI_PASSWORD "mastero101"

int a=0; //variable of value of analog read pin
char t[4];//variable to send text to the oled display   
int b=0; //variable of value of analog read pin
char h[4];//variable to send text to the oled display   
int c=0; //variable of value of analog read pin
char p[4];//variable to send text to the oled display       
int hi=0;  
int tf=0;
int hic=0;

// Insert Firebase project API Key
#define API_KEY "AIzaSyBp7UGCt31inIEFavIztAKZ0ZDM7pC2vcc"

// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "1234@gmail.com"
#define USER_PASSWORD "123456"

// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://bme280-e3ead-default-rtdb.firebaseio.com/"

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
String tempPath = "/temperature";
String humPath = "/humidity";
String presPath = "/pressure";
String relPath = "/temprel";
String timePath = "/timestamp";

// Parent Node (to be updated in every loop)
String parentPath;

FirebaseJson json;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Variable to save current epoch time
int timestamp;

// BME280 sensor
Adafruit_BME280 bme; // I2C
float temperature;
float humidity;
float pressure;
float altitude;

// Timer variables (send new readings every three minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 300000;

// Initialize BME280
void initBME(){
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

// Initialize WiFi
void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
}

// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

void setup(){
  Serial.begin(115200);

  // Initialize BME280 sensor
  initBME();
  initWiFi();
  timeClient.begin();
  u8g2.begin();
  u8g2.setContrast(16);

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);

  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";
}

void loop(){
  //Imp OLED i2c

  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  a=hic; 
  b=bme.readHumidity(); 
  c=pressure; 
  sprintf(t,"%04d",a); 
  sprintf(h,"%04d",b);
  sprintf(p,"%04d",c);
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_inr16_mf); // choose a suitable font
  u8g2.drawStr(10,15,t);  // write something to the internal memory
  u8g2.drawStr(10,40,h);
  u8g2.drawStr(10,65,p);
  u8g2.drawStr(70,15," C");
  u8g2.drawStr(70,40," %");
  u8g2.drawStr(70,65," hPa");
  u8g2.sendBuffer();   
  tf = temperature*1.8+32;
  hi = 0.5 * (tf + 61.0 + ((tf - 68.0) * 1.2) +
              (humidity * 0.094));
   if (hi > 79);
      hi = -42.379 + 2.04901523 * tf + 10.14333127 * humidity +
         -0.22475541 * tf * humidity +
         -0.00683783 * pow(tf, 2) +
         -0.05481717 * pow(humidity, 2) +
         0.00122874 * pow(tf, 2) * humidity +
         0.00085282 * tf * pow(humidity, 2) +
         -0.00000199 * pow(tf, 2) * pow(humidity, 2);
  hic = (hi - 32) /1.8;
  
  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

    //Get current timestamp
    timestamp = getTime();
    Serial.print ("time: ");
    Serial.println (timestamp);

    parentPath= databasePath + "/" + String(timestamp);

    json.set(tempPath.c_str(), String(bme.readTemperature()));
    json.set(humPath.c_str(), String(bme.readHumidity()));
    json.set(presPath.c_str(), String(bme.readPressure()/100.0F));
    json.set(relPath.c_str(), String(hic));
    json.set(timePath, String(timestamp));
    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
  }
}
