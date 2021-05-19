
#include "ESP32MotorControl.h" 

//DRV8833 Motor control setting
int speed = 100;
const int MOTOR_GPIO_IN1 = 12;
const int MOTOR_GPIO_IN2 = 13;
const int MOTOR_GPIO_IN3 = 16;
const int MOTOR_GPIO_IN4 = 17;
ESP32MotorControl MotorControl = ESP32MotorControl();

//Touch control
const int t_level=60;
#define left_t 4
#define right_t 14
int left_val;
int right_val;

void setup() {

  Serial.begin(115200);
//Setup Motor Sheild 
  MotorControl.attachMotors(MOTOR_GPIO_IN1, MOTOR_GPIO_IN2, MOTOR_GPIO_IN3, MOTOR_GPIO_IN4);
}

void loop() {
  left_val=touchRead(left_t);
  right_val=touchRead(right_t);
//Serial.println(left_val);
//Serial.println(right_val);
if (left_val < t_level) {
  MotorControl.motorForward(0, speed);
  }
if (right_val < t_level) {
  MotorControl.motorReverse(0, speed);
  }
delay(500);
MotorControl.motorsStop(); 
}
