//for this to flash, you need to change the processor to ATmega328P (old bootloader) in the tools menu
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS.h>
#include <Servo.h>
Servo thruster;
void setup() {
Serial.begin(9600);
//lsm.begin();
thruster.attach(9); //pin d9 on arduino

thruster.writeMicroseconds(1500);  // Neutral position (motor stopped), needed for esc initialization
delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
thruster.writeMicroseconds(1510);
delay(1000);
}
