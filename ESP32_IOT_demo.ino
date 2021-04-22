/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
#include "DHT.h"
#define green_led 19
#define red_led 12
#define button 18
#define DHTPIN 4
#define ldr 36
#define DHTTYPE DHT11
int buttonState = 0; 
DHT dht(DHTPIN, DHTTYPE);

// the setup function runs once when you press reset or power the board
void setup() {
    //Initialize serial and wait for port to open:
  Serial.begin(115200);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(button, INPUT);
  Serial.println(F("DHT11 test!"));

  dht.begin();

}

// the loop function runs over and over again forever
void loop() {

  digitalWrite(green_led, LOW); // turn the Green  LED off (LOW is the voltage level)
  delay(100);                       // wait for a second

  digitalWrite(green_led, HIGH); // turn the Green  LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second


  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
//  buttonState = digitalRead(button);
//  Serial.print("Current Light level ");
//  Serial.println(analogRead(ldr));
//  if (buttonState == LOW) {
    // turn LED on:
//    digitalWrite(red_led, HIGH);
//    Serial.println("button is ressed!");
//  } else {
    // turn LED off:
//    digitalWrite(red_led, LOW);
//  }
//  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
//  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
//  float f = dht.readTemperature(true);
//  if (isnan(h) || isnan(t) || isnan(f)) {
//   Serial.println(F("Failed to read from DHT sensor!"));
//   return;
//  }
    // Compute heat index in Fahrenheit (the default)
//  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
//  float hic = dht.computeHeatIndex(t, h, false);

//  Serial.print(F("Humidity: "));
//  Serial.print(h);
//  Serial.print(F("%  Temperature: "));
//  Serial.print(t);
//  Serial.print(F("°C "));
//  Serial.print(f);
//  Serial.print(F("°F  Heat index: "));
//  Serial.print(hic);
//  Serial.print(F("°C "));
//  Serial.print(hif);
//  Serial.println(F("°F"));
}
