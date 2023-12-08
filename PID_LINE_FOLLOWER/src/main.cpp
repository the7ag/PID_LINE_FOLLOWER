#include <L298N.h>
#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "IOT"
#define WIFI_PASSWORD "77777777"

// Insert Firebase project API Key
#define API_KEY "AIzaSyCZHt0LZpn2yPbaPEe71zM3_QFxV8aIvEI"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://alarm-e6853-default-rtdb.firebaseio.com/" 

//Define Firebase Data object
FirebaseData fbdoplay;
//Define firebase auth and config
FirebaseAuth auth;
FirebaseConfig config;
//Millis for timers
unsigned long startDataPrevMillis=0;
//Sensor count
#define SENSOR_COUNT 5
//Motor Driver Pins
int M1F=13,M1B=12,M2F=14,M2B=27,M1S=26,M2S=25;
//Buzzer Pin
int buzzer = 19;
//IR Pins
int IR[SENSOR_COUNT]={33,18,4,5,32};
//Sensor Values
int sensorValues[SENSOR_COUNT];
//PID Calculations
int P;
int I;
int D;
int lastError = 0;
// Best Selected Variables with a speed of 39% power of 9v
float Kp = 0.1;
float Ki = 0.00005;
float Kd = 0.4;
// Start variable from the firebase
int start=0;
// Signup check
int signupOK = false;
// Instance of L298N object
L298N motor1(M1S, M1F, M1B);
L298N motor2(M2S, M2F, M2B);
// Function prototypes
int readPosition();
void readStart();
void PID_control();
void movement(int speedA, int speedB);

void setup() {
//Set sensors as input
  for(int i=0;i<SENSOR_COUNT;i++)
  {
    pinMode(IR[i], INPUT);
  }
  //Buzzer as output
  pinMode(buzzer,OUTPUT);
  //Serial For debuggeing
  Serial.begin(115200);
  //Wifi initalization
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
   }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  /* Assign the api key (required) */
  config.api_key = API_KEY;
  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;
  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  //Check if the start == 0 no action and keep reading it from the DB
  if (start==0)
  {
    digitalWrite(buzzer,LOW);
    readStart();
    motor1.stop();
    motor2.stop();
  }
  // If we started run the algorithim and the alarm
  if(start==1)
  {
    digitalWrite(buzzer,HIGH);
    PID_control();
  }
}
//Read Start from the DB
void readStart()
{
 if (Firebase.ready() && signupOK && (millis() - startDataPrevMillis > 300 || startDataPrevMillis == 0)) {
    startDataPrevMillis = millis();
    //Get request from the fire base with the key=/play 
    //                                        value=/play
    if (Firebase.RTDB.getInt(&fbdoplay, "/play/play")) {
      //Check if the json object has an int data
      if (fbdoplay.dataType() == "int") {
        //Read it and save it in the start var
       start=fbdoplay.intData();
       Serial.print(" state Value : ");
       Serial.println(start);
      }
    }
    else {
      Serial.println(fbdokd.errorReason());
    }
 }
}
//This function reads the position of the IR sensor and multiplyes it by 1000
int readPosition()
{
  int tempValue=0;
  int tempSum=0;
  //Loop on all the sensors
  for (int i=0; i<SENSOR_COUNT; i++)
  {
    sensorValues[i]=digitalRead(IR[i]);
    //Sum all it's readings
    tempSum+=sensorValues[i];
    // Read it's position multiply it by 1000
    tempValue+=(sensorValues[i]*(i)*1000);
  }
  //To avoid division by 0
  if(tempSum != 0){
    tempValue/=tempSum;
  }
  // else if(tempSum==0)
  // {
  //   return 2000;
  // }
  return tempValue;
  }
//This function uses the PID control algorithim and increases the speed of one motor and decreases the
// speed of the other
void PID_control() {
  //The poisiton ranges from 0 to 4000
  uint16_t positionLine = readPosition();
  // The error is ragne is 2000 to -2000
  int error = 2000 - positionLine;
    Serial.println("Error: " + error);
  /* p =the error
     I = Sum of errors (area under the curve)
     D = Error - previous error (d/dt)
  */
  P = error;
  I = error + I;
  D = error - lastError;
  //Update last eror value for next D value
  lastError = error;
  //Multiply every constant with the PID values to get the speed change
  int motorSpeedChange = P*Kp + I*Ki + D*Kd;
  Serial.println("Speed: " + motorSpeedChange);
  //Added to one side and remove it from the other side
  // Speed change can be positive or negative
  int motorSpeedA = 100 + motorSpeedChange;
  int motorSpeedB = 100 - motorSpeedChange;
  //Constrain the speed ranges too -50 and 100
  if (motorSpeedA > 100) {
    motorSpeedA = 100;
  }
  if (motorSpeedB > 100) {
    motorSpeedB = 100;
  }
  if (motorSpeedA < -50) {
    motorSpeedA = -50;
  }
  if (motorSpeedB < -50) {
    motorSpeedB = -50;
  }
  //Call the movement
  movement(motorSpeedA, motorSpeedB);
}
// This function just changes the dir of the motor if it has a negative speed
void movement(int speedA, int speedB) {
  if (speedA < 0) {
    speedA = 0 - speedA;
    motor1.forward();
  }
  else {
    motor1.backward();
  }
  if (speedB < 0) {
    speedB = 0 - speedB;
    motor2.forward();
  }
  else {
      motor2.backward();
    }
  motor1.setSpeed(speedA);
  motor2.setSpeed(speedB);
}