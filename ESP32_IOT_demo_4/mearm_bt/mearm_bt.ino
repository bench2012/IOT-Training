#include <Ps3Controller.h>
#include <ESP32Servo.h>

Servo servoL; 
Servo servoR;
Servo servoB;
Servo servoC;

int rX;
int rY;
int lX;
int lY;

int posR = 90;
int posL = 90;
int posB = 90;
int posC = 90;

void notify()
{
 lX =(Ps3.data.analog.stick.lx);
 lY =(Ps3.data.analog.stick.ly);
 rX =(Ps3.data.analog.stick.rx);
 rY =(Ps3.data.analog.stick.ry);

 Serial.print(lX);
 Serial.print(" ");
 Serial.print(lY);
 Serial.print(" ");
 Serial.print(rX);
 Serial.print(" ");
 Serial.println(rY);

 if(lY < -5 && posR < 180)
 {
  posR++;
  servoR.write(posR);
  delay(10);
 }
 if(lY > 5 && posR > 0)
 {
  posR--;
  servoR.write(posR);
  delay(10);
 }

 if(rY < -5 && posL < 180)
 {
  posL++;
  servoL.write(posL);
  delay(10);
 }
 if(rY > 5 && posL > 0)
 {
  posL--;
  servoL.write(posL);
  delay(10);
 }
 
 if(rX < -5 && posB < 180)
 {
  posB++;
  servoB.write(posB);
  delay(10);
 }
 if(rX > 5 && posB > 0)
 {
  posB--;
  servoB.write(posB);
  delay(10);
 }

 if( abs(Ps3.event.analog_changed.button.l2) ){
        servoC.write(120);
       }
   

 if( abs(Ps3.event.analog_changed.button.r2) ){
        servoC.write(90);
       }
}

void onConnect(){
    Serial.println("Connected.");
}

void setup()
{
    Serial.begin(115200);
    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("54:27:1E:E5:85:DB");
    Serial.println("Ready.");
    servoL.attach(27);  
    servoR.attach(16);
    servoB.attach(17);
    servoC.attach(25);
    servoL.write(posL);
    servoR.write(posR);
    servoB.write(posB);
    servoC.write(posC);
}

void loop()
{
    if(!Ps3.isConnected())
        return;
    delay(2000);
}
