#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS.h>
#include <Servo.h>
Servo thruster;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
//lsm.begin();
thruster.attach(A3);
}

void loop() {
  // put your main code here, to run repeatedly:
thruster.write(12);
}
