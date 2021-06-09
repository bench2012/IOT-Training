/*********
Complete rewritten (GUi) using ESPUI
*********/

#include <ESPUI.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include "ESP32MotorControl.h" 

// MotorControl instance

ESP32MotorControl MotorControl = ESP32MotorControl();

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "ssid";
const char* password = "password";

const char* ap_ssid = "iot_ap";
const char* ap_password = "cubie12345";
boolean WiFiUp = false;

//Sleep timer 
const int to_sleep_time = 60;
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;
const int led = 2; //sleep LED
const int wake_pin = 25;

//Wifi connect wait timer
const int Wifi_Retry = 3;

//DRV8833 Motor control setting
const int speed = 120;
const int MOTOR_GPIO_IN1 = 19;
const int MOTOR_GPIO_IN2 = 18;
const int MOTOR_GPIO_IN3 = 16;
const int MOTOR_GPIO_IN4 = 17;
const int drv8833_sleep = 27; // DRV8833 sleep pin

//Beep setting
unsigned int hibeep = 500;
unsigned int lowbeep = 100;
unsigned int beeps = 10;


#define BUZZER_PIN          12

#ifdef    ARDUINO_ARCH_ESP32

#define SOUND_PWM_CHANNEL   0
#define SOUND_RESOLUTION    8 // 8 bit resolution
#define SOUND_ON            (1<<(SOUND_RESOLUTION-1)) // 50% duty cycle
#define SOUND_OFF           0                         // 0% duty cycle

// LED Eyes
#define NUM_LEDS 2
#define DATA_PIN 15


void tone(int pin, int frequency, int duration)
{
  ledcSetup(SOUND_PWM_CHANNEL, frequency, SOUND_RESOLUTION);  // Set up PWM channel
  ledcAttachPin(pin, SOUND_PWM_CHANNEL);                      // Attach channel to pin
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_ON);
  delay(duration);
  ledcWrite(SOUND_PWM_CHANNEL, SOUND_OFF);
}

#endif

void roboControl(Control *sender, int value) {
  switch (value) {

  case P_LEFT_DOWN:
    MotorControl.motorForward(1, speed);
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("left");
    break;

  case P_LEFT_UP:
    MotorControl.motorsStop();
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("stop");
    break;

  case P_RIGHT_DOWN:
    MotorControl.motorForward(0, speed);
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("right");
    break;

  case P_RIGHT_UP:
    MotorControl.motorsStop();
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("stop");
    break;

  case P_FOR_DOWN:
    MotorControl.motorForward(0, speed);
    MotorControl.motorForward(1, speed);
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("forward");
    break;

  case P_FOR_UP:
    MotorControl.motorsStop();
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("stop");
    break;

  case P_BACK_DOWN:
    MotorControl.motorReverse(0, speed);
    MotorControl.motorReverse(1, speed);
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("back");
    break;

  case P_BACK_UP:
    MotorControl.motorsStop();
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("stop");
    break;

  case P_CENTER_DOWN:
    MotorControl.motorsStop();
    startTimer = true;
    lastTrigger = millis();
    digitalWrite(led, HIGH);
    Serial.println("stop");
    break;

  case P_CENTER_UP:
    Serial.print("center up");
    break;

  Serial.print(" ");
  Serial.println(sender->id);
  }
}

void setup() {
  ESPUI.setVerbosity(Verbosity::VerboseJSON);
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.setAutoConnect(false);
  Serial.printf("Scanning for %s\r\n", ssid); // if WiFi/LAN is available
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
   //Setup Status LED Off is sleeping
  pinMode(led, OUTPUT);
  //Setup Wake/sleep button pull dowen
  pinMode(wake_pin, INPUT_PULLDOWN);
  
  //Setup DRV8833 sleep pin
  pinMode(drv8833_sleep, OUTPUT);
  gpio_pulldown_en(GPIO_NUM_27);
  gpio_pulldown_en(GPIO_NUM_25);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25 ,1);


  //Wait for Wifi. If nothing after 20 loop, change to AP mode
  for (int i = 0; i < n; ++i) {
    if (WiFi.SSID(i) == ssid) {
      WiFiUp = true;
      WiFi.mode(WIFI_AP_STA); // LAN and AP and UDP clients
      //WiFi.config(ip, gateway, subnet); // LAN fixed IP
      WiFi.begin(ssid, password); // connect to LAN with credentials
      Serial.printf("Found %s, trying to connect ", ssid);
      break;
    }
    delay(10);
  }
  connectWiFi();
 

 

  // OTA Stuff
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());

  //Setup Motor Sheild 
  MotorControl.attachMotors(MOTOR_GPIO_IN1, MOTOR_GPIO_IN2, MOTOR_GPIO_IN3, MOTOR_GPIO_IN4);
  
  digitalWrite(led, HIGH); // Sleep LED On for awake
  digitalWrite(drv8833_sleep, HIGH); //enable drv8833
  
  //Start ESPUI
  ESPUI.padWithCenter("Robo Conrtol Pad with center", &roboControl, ControlColor::Sunflower); 
  ESPUI.begin("Robo Control");

}

void connectWiFi() {
  if (WiFiUp) {
    byte w8 = 0;
    while (WiFi.status() != WL_CONNECTED && w8++ < Wifi_Retry) {
      delay(333); // try for 5 seconds
      Serial.print(">");
    }
    Serial.printf("\r\n");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\tConnected to %s IP address %s strength %d%%\r\n", ssid, WiFi.localIP().toString().c_str(), 2 * (WiFi.RSSI() + 100));
    WiFi.setAutoReconnect(false);
    //retryCounter = 0; // reset counter when connected

    //Two hi beep for Station mode
    tone(BUZZER_PIN,hibeep,100);
    delay(200);
    tone(BUZZER_PIN,hibeep,100);
  } else {
    WiFi.mode(WIFI_AP); // drop station mode if LAN/WiFi is down
  //  WiFi.softAP(ap_ssid, ap_password);
    WiFi.softAP(ap_ssid);
    Serial.printf("\tLAN Connection failed\r\n\tTry %s AP with IP address %s\r\n", ap_ssid, 
    WiFi.softAPIP().toString().c_str());
    
  //Two low beep for AP mode
    tone(BUZZER_PIN,lowbeep,100);
    delay(200);
    tone(BUZZER_PIN,hibeep,300);  
  }
 }


void sleep_display() {

// Two Red Blick and Beep One hi One low for sleep
     digitalWrite(led, LOW);
     digitalWrite(drv8833_sleep, LOW); //Put drv8833 to sleep first

     tone(BUZZER_PIN,hibeep,100);
     tone(BUZZER_PIN,lowbeep,800);
     delay(500); 

}

void sleep_detect() {
     now = millis();
  if (digitalRead(wake_pin) == 1) { //if press the sleep/wake button
     startTimer = false;
     Serial.println("Going to sleep now on button");
     
     delay(1000);
// Two Beep One hi One low for sleep
     sleep_display();
     esp_deep_sleep_start(); //Then goto sleep
  }
  
  if(startTimer && (now - lastTrigger > (to_sleep_time*1000))) {
    
    startTimer = false;
    Serial.println("Going to sleep now");
    delay(1000);
    sleep_display();
    esp_deep_sleep_start(); //Then goto sleep
  }
}


void loop() {

// Turn off the LED after the number of seconds defined in the timeSeconds variable or button press
   startTimer = true;
   sleep_detect();
//   Serial.println((now - lastTrigger)/1000);
    ArduinoOTA.handle(); 
}
