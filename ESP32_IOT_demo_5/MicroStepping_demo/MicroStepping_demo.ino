/*
 * Microstepping demo
 *
 * This requires that microstep control pins be connected in addition to STEP,DIR
 *
 * Copyright (C)2015 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 20
#define RPM 30
int Step_size = 1;
int Step = 1;
int Max_Step = 100;
int dir = 1;
int   inbyte = 0 ;

#define DIR 12
#define STEP 13
#define SLEEP 13 // optional (just delete SLEEP from everywhere if not used)

/*
 * Choose one of the sections below that match your board
 */

#include "A4988.h"
#define MS1 19
#define MS2 23
#define MS3 5
A4988 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, MS1, MS2, MS3);


void setup() {
    Serial.begin(115200);
    /*
     * Set target motor RPM.
     */
    stepper.begin(RPM);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    stepper.setEnableActiveState(LOW);
    stepper.enable();
    
    // set current level (for DRV8880 only). 
    // Valid percent values are 25, 50, 75 or 100.
    // stepper.setCurrent(100);
}

void menuRun()
{
  // ConOut.print("Choose Step Size");
  if (Serial.available())   inbyte = Serial.read(); //Serial input available
  switch (inbyte)
  {

    //====(Serial Menu)======
    case 'f': //Full Step
      Step_size=1;
      Serial.println(Max_Step*Step_size);
      break;
 
    case 'h': //Half Step
      Step_size=2;
      Serial.println(Max_Step*Step_size);
      break;
 
    case 'q': // 1/4 Step
      Step_size=4;
      Serial.println(Max_Step*Step_size);
      break;
      
    case 'e': // 1/8 Step
      Step_size=8;
      Serial.println(Max_Step*Step_size);
      break;
      
    case 's': // 1/16 Step
      Step_size=16;
      Serial.println(Max_Step*Step_size);
      break;
       
  } //end switch (inbyte)

  inbyte = 0 ;

}
void loop() {

    menuRun();

    stepper.setMicrostep(Step_size);  // Set microstep mode 

    stepper.move(Max_Step*Step_size); // Forward step
    delay(500);
    stepper.move(-Max_Step*Step_size); // Backward step

}
