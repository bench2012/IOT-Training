#include <dataControlsJS.h>
#include <dataGraphJS.h>
#include <dataIndexHTML.h>
#include <dataNormalizeCSS.h>
#include <dataSliderJS.h>
#include <dataStyleCSS.h>
#include <dataTabbedcontentJS.h>
#include <dataZeptoJS.h>
  
/*********
Complete rewritten (GUi) using ESPUI
*********/

#include <ESPUI.h>
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
#include "DHT.h"


#define green_led 19
#define red_led 12
//#define led 2
#define button_1 18
#define DHTPIN 4
#define ldr 36
#define DHTTYPE DHT11
bool buttonState = false; 
int light_level = 0;
int statusLabelId;
int graphId;
int testSwitchId;
int HumidityLabelId;
int TemperatureLabelId;
DHT dht(DHTPIN, DHTTYPE);



// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "macross2010";
const char* password = "qswdefrgthyjukil";

const char* ap_ssid = "iot_ap";
const char* ap_password = "iot12345";
boolean WiFiUp = false;

//Wifi LED

const int led = 2; //Wifi LED

//Wifi connect wait timer
const int Wifi_Retry = 3;

void numberCall(Control *sender, int type) { Serial.println(sender->value); }

void switch_green(Control *sender, int value) {
  switch (value) {
  case S_ACTIVE:
    digitalWrite(green_led,HIGH);
    Serial.print("Switch On:");
    break;

  case S_INACTIVE:
    digitalWrite(green_led,LOW);
    Serial.print("Switch Off");
    break;
  }
}

void switch_red(Control *sender, int value) {
  switch (value) {
  case S_ACTIVE:
    digitalWrite(red_led,HIGH);
    Serial.print("Switch On:");
    break;

  case S_INACTIVE:
    digitalWrite(red_led,LOW);
    Serial.print("Switch Off");
    break;
  }
  Serial.print(" ");
  Serial.println(sender->id);
}




void setup() {
  ESPUI.setVerbosity(Verbosity::Verbose);

  Serial.begin(115200);
  Serial.println("Booting");
  pinMode(led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(button_1, INPUT);
  dht.begin();

  //All the Wifi connection bit 
  WiFi.setAutoConnect(false);
  Serial.printf("Scanning for %s\r\n", ssid); // if WiFi/LAN is available
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
   //Setup Status LED Off is sleeping
  pinMode(led, OUTPUT);

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

 
  //Start ESPUI
  HumidityLabelId = ESPUI.label("Humidity:", ControlColor::Emerald, "0");
  TemperatureLabelId = ESPUI.label("Temperature:", ControlColor::Emerald, "0");
  testSwitchId = ESPUI.switcher("Button one", &switch_red, ControlColor::Alizarin, false);
  ESPUI.switcher("Green LED ", &switch_green, ControlColor::Wetasphalt, false);
  graphId = ESPUI.graph("Light Level", ControlColor::Wetasphalt);     

  ESPUI.begin("IOT Dashboard");

}

void connectWiFi() {
  if (WiFiUp) {
    byte w8 = 0;
    while (WiFi.status() != WL_CONNECTED && w8++ < Wifi_Retry) {
      delay(333); // try for 5 seconds
      Serial.print(">");
      Wifi_connecting_blink();
    }
    Serial.printf("\r\n");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\tConnected to %s IP address %s strength %d%%\r\n", ssid, WiFi.localIP().toString().c_str(), 2 * (WiFi.RSSI() + 100));
    WiFi.setAutoReconnect(false);
    digitalWrite(led, HIGH);

  } else {
    WiFi.mode(WIFI_AP); // drop station mode if LAN/WiFi is down
  //  WiFi.softAP(ap_ssid, ap_password);
    WiFi.softAP(ap_ssid);
    Serial.printf("\tLAN Connection failed\r\n\tTry %s AP with IP address %s\r\n", ap_ssid, WiFi.softAPIP().toString().c_str());
    Wifi_connecting_blink();
  }
 }

void Wifi_connecting_blink() {

    delay(500);
    Serial.print(".");
     digitalWrite(led,HIGH);
    
        delay(800); 
     digitalWrite(led,LOW);

        delay(500);


}



void loop() {
  buttonState = digitalRead(button_1);
  Serial.print("Current Light level ");
  light_level=analogRead(ldr);
  Serial.println(light_level);
  if (buttonState == LOW) {
    // turn LED on:
    digitalWrite(red_led, HIGH);
    Serial.println("button is Pressed!");
    buttonState=!buttonState;
  } else {
    // turn LED off:
    digitalWrite(red_led, LOW);
    buttonState=!buttonState;
  }

  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
float t = dht.readTemperature();
float f = dht.readTemperature(true);
float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
float hic = dht.computeHeatIndex(t, h, false);
if (isnan(h) || isnan(t) || isnan(f)) {
   Serial.println(F("Failed to read from DHT sensor!"));
   return;
  }
    // Compute heat index in Fahrenheit (the default)


  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("??C "));
  Serial.print(f);
  Serial.print(F("??F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("??C "));
  Serial.print(hif);
  Serial.println(F("??F"));


    ESPUI.print(HumidityLabelId, String(h));
    ESPUI.print(TemperatureLabelId, String(t));
    ESPUI.updateSwitcher(testSwitchId, buttonState);
    ESPUI.addGraphPoint(graphId, light_level);


    ArduinoOTA.handle(); 
}
