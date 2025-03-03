//for this to flash, you need to change the processor to ATmega328P (old bootloader) in the tools menu
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Servo.h>
#include <QuadClass_LSM6DSOX.h> //I am reusing a bunch of code from a quadcopter project I did to read IMU data
Servo thrusterLeft;
Servo thrusterRight;
Servo thrusterTopLeft;
Servo thrusterTopRight;

  float pitch=0.0; 
  float pitch_rate = 0.0;
  float roll=0.0;
  float yaw = 0.0;
  float target_yaw = 0.0;
  float yawerror=0;
  float oldyawerror=0;
  float integralyaw = 0;
  float derivativeyaw=0;

  QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
  Adafruit_Simple_AHRS *ahrs = NULL;
  Adafruit_Sensor *_accel = NULL;
  Adafruit_Sensor *_gyro = NULL;
  Adafruit_Sensor *_mag = NULL; 

  #define RAD_TO_DEG 57.295779513082320876798154814105
  float gain = 0.98; 

  double cf_pitch = 0.0;
  double cf_roll = 0.0;
  double pitch_offset = 0.0;//1.88+0.51;
  double roll_offset = 0.0; 
  double yaw_offset = 0.0;
  double yaw_corrected = 0.0;
  double pitch_corrected = 0.0; 
  double roll_corrected = 0.0;
  double gyro_angle_yaw = 0.0;
  double gyro_raw_yaw = 0.0;

  int motorRight = 1500;
  int motorLeft = 1500;
  int topRight = 1500;
  int topLeft = 1500;

  int16_t thrust = 0;
  int16_t thrustmotor=0;
  float desiredpitch= 0;
  float desiredyaw = 0;

  float oldpitcherror=0;
  float pitcherror=0;
  float integralpitch=0;
  float derivativepitch=0;
  float pitchcontrol=0;
  float yawcontrol=0;
  float Kp=0.2;
  float Ki=0.0;
  float Kd=0.02;
  float Yp=0.1;
  float Yi=0.0;
  float Yd=0.02;
  
  float dt=0.0;

void setupSensor()
{


 if (!lsm.begin_I2C()) {
   // Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  _accel = lsm.getAccelerometerSensor();
  _gyro = lsm.getGyroSensor();


  ahrs = new Adafruit_Simple_AHRS(_accel, _mag, _gyro);
#
}

// Code to receive keyboard input
char receivedChar;
void recvOneChar() {
  if (Serial.available() > 0) {
      receivedChar = Serial.read();
      //Serial.println(receivedChar);
      desiredPos(receivedChar);
  }
}
void interpretKey(char key) {
  if (key == 'w') {
    thrusterLeft.writeMicroseconds(1550);
    //thrusterRight.writeMicroseconds(1550);
  } else if (key == 'a') {
    thrusterRight.writeMicroseconds(1550);
  } else if (key == 's') {
    //thrusterTopRight.writeMicroseconds(1550);
    thrusterRight.writeMicroseconds(1450);
  } else if (key == 'd') {
    //thrusterTopLeft.writeMicroseconds(1550);
    thrusterLeft.writeMicroseconds(1450);
  } else if (key == 'q') {
    thrusterLeft.writeMicroseconds(1500);
    thrusterRight.writeMicroseconds(1500);
    thrusterTopRight.writeMicroseconds(1500);
    thrusterTopLeft.writeMicroseconds(1500);
  }
}

void desiredPos(char key){
  if (key=='e'){
  thrust=thrust+1;
  //digitalWrite(LED_BUILTIN, HIGH);
  } 
  if (key=='w'){
  thrust = thrust-1;
  //digitalWrite(LED_BUILTIN, HIGH);
  }
  if (key=='q'){
    thrust = 0;
  }
  if (key=='d'){
  desiredpitch=desiredpitch+1;}
  if (key=='s'){
    desiredpitch=desiredpitch-1;
  }
  if (key=='a'){
  desiredpitch=0;}
  if (key=='l'){
    desiredyaw=desiredyaw+1;
  }
  if (key == 'k') {
    desiredyaw=desiredyaw-1;
  }
  if (key == 'j'){
    desiredyaw=0;
  }
  if (key == 'i') {
      pitch_offset=-cf_pitch;
  }
  if (key == 'o') {
    yaw_offset=-gyro_angle_yaw;
  }
}


void setup() {
Serial.begin(9600);
pinMode(LED_BUILTIN, OUTPUT);

setupSensor();

thrusterLeft.attach(2); //pin d2
thrusterRight.attach(3);
thrusterTopLeft.attach(4);
thrusterTopRight.attach(5);

thrusterLeft.writeMicroseconds(1500);  // Neutral position (motor stopped), needed for esc initialization
//1000 is full backwards, 1500 is stopped, 2000 is full forwards
thrusterRight.writeMicroseconds(1500);
thrusterTopLeft.writeMicroseconds(1500);
thrusterTopRight.writeMicroseconds(1500);

delay(2000);
}
unsigned long  last = millis();
void loop() {
  quad_data_t orientation;
  unsigned long  now = millis();
  dt = (now - last);

if (ahrs->getQuadOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    //Serial.print(now - last);
    pitch = orientation.pitch;
   //("rawpitch:");
   //Serial.println(pitch);

    pitch_rate = orientation.pitch_rate;
    //Serial.print(" raw_pitch_rate:");
    //Serial.println(pitch_rate);
    //yaw=orientation.yaw;

    roll = orientation.roll;
    
    sensors_event_t gyro_event;
    _gyro->getEvent(&gyro_event);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm.setGyroDataRate(LSM6DS_RATE_208_HZ);

    sensors_event_t accel;
    _accel->getEvent(&accel);
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm.setAccelDataRate(LSM6DS_RATE_208_HZ);
    lsm.setAccelCompositeFilter(LSM6DS_CompositeFilter_LPF2, LSM6DS_CompositeFilter_ODR_DIV_800);
    double gyro_raw_pitch = gyro_event.gyro.y; //.z was there initially, may need pid change
    double gyro_raw_roll = gyro_event.gyro.x;
    gyro_raw_yaw = gyro_event.gyro.z;

    
    double gyro_angle_pitch = cf_pitch + (gyro_raw_pitch * RAD_TO_DEG)*dt*0.001;
    double gyro_angle_roll = cf_roll + (gyro_raw_roll * RAD_TO_DEG)*dt*0.001;

    gyro_angle_yaw = gyro_angle_yaw + (gyro_raw_yaw*RAD_TO_DEG*dt*0.001);

    cf_pitch = (gain * gyro_angle_pitch) + (1.0-gain)*pitch;
    cf_roll = (gain * gyro_angle_roll) + (1.0-gain)*roll;
  


/*
    Serial.print("cf_pitch:");
    Serial.println(cf_pitch);

    Serial.print("cf_roll:");
    Serial.println(cf_roll);

    Serial.print("gyro_angle_yaw:");
    Serial.println(gyro_angle_yaw);
*/
  }
/*
 if(cf_roll>50)
{
thrusterLeft.writeMicroseconds(1600);//esc struggles with lower throttle
}
else if(cf_roll<=-50){
  thrusterRight.writeMicroseconds(1600);
}
else{
  thrusterLeft.writeMicroseconds(1500);
  thrusterRight.writeMicroseconds(1500);
} 
*/

recvOneChar();
PID();
//interpretKey(receivedChar);

  last = now;
} 


void PID(){
  pitch_corrected=cf_pitch+pitch_offset;
  yaw_corrected=gyro_angle_yaw+yaw_offset;

  pitcherror = desiredpitch - pitch_corrected;
  yawerror = desiredyaw-yaw_corrected;

  integralpitch=integralpitch+pitcherror*dt*0.001;
  derivativepitch=(pitcherror-oldpitcherror)/dt;

  integralyaw=integralyaw+yawerror*dt*0.001;
  derivativeyaw=(yawerror-oldyawerror)/dt;

  pitchcontrol=Kp*pitcherror+Ki*integralpitch+Kd*derivativepitch;
  yawcontrol=Yp*yawerror+Yi*integralyaw+Yd*derivativeyaw;
  
  oldpitcherror=pitcherror;
  oldyawerror=yawerror;
  motorcontrol();
}


void motorcontrol(){
  //Serial.print("thrust: ");
  //Serial.println(thrust);
  thrustmotor=constrain(thrust,-15,15);
  thrustmotor=map(thrust,-15,15,1200,1800);
  //motorRight=thrustmotor;
  Serial.print('pitchcontorl: ');
  Serial.println(pitchcontrol);
  
  pitchcontrol=constrain(pitchcontrol,-10,10);
  pitchcontrol=map(pitchcontrol,-10,10,-250,250);
  yawcontrol=constrain(yawcontrol,-10,10);
  yawcontrol=map(yawcontrol,-10,10,-250,250);


  motorRight=thrustmotor+pitchcontrol+yawcontrol;
  motorLeft=thrustmotor+pitchcontrol-yawcontrol;
  topRight=thrustmotor-pitchcontrol+yawcontrol;
  topLeft=thrustmotor-pitchcontrol-yawcontrol;
  motorRight=constrain(motorRight,1200,1800);
  motorLeft=constrain(motorLeft,1200,1800);
  topRight=constrain(topRight,1200,1800);
  topLeft=constrain(topLeft,1200,1800);
  thrusterRight.writeMicroseconds(motorRight);
  thrusterLeft.writeMicroseconds(motorLeft);
  thrusterTopRight.writeMicroseconds(topRight);
  thrusterTopLeft.writeMicroseconds(topLeft);
  Serial.print("rightThrust: ");
  Serial.println(motorRight);
}