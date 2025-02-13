//for this to flash, you need to change the processor to ATmega328P (old bootloader) in the tools menu
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Simple_AHRS.h>
#include <Servo.h>
#include <QuadClass_LSM6DSOX.h> //I am reusing a bunch of code from a quadcopter project I did to read IMU data
Servo thruster;

  float pitch=0.0; 
  float pitch_rate = 0.0;
  float roll=0.0;
  float yaw = 0.0;
  float target_yaw = 0.0;
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
  double pitch_corrected = 0.0; 
  double roll_corrected = 0.0;
  double gyro_angle_yaw = 0.0;

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

void setup() {
Serial.begin(9600);

setupSensor();

thruster.attach(9); //pin d9 on arduino

thruster.writeMicroseconds(1500);  // Neutral position (motor stopped), needed for esc initialization
//1000 is full backwards, 1500 is stopped, 2000 is full forwards
delay(2000);
}
unsigned long  last = millis();
void loop() {
  quad_data_t orientation;
  unsigned long  now = millis();
  float dt = (now - last);

if (ahrs->getQuadOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    //Serial.print(now - last);
    pitch = orientation.pitch;
   Serial.print("rawpitch:");
   Serial.println(pitch);

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
    double gyro_raw_yaw = gyro_event.gyro.z;

  //Serial.print("\t\tAccel X: ");
  //Serial.print(accel.acceleration.x);
  //Serial.print(" \tY: ");
  //Serial.println(accel.acceleration.y);

   Serial.print("gyro_raw_pitch:");
   Serial.println(gyro_raw_pitch*RAD_TO_DEG);

    
    double gyro_angle_pitch = cf_pitch + (gyro_raw_pitch * RAD_TO_DEG)*dt*0.001;
    double gyro_angle_roll = cf_roll + (gyro_raw_roll * RAD_TO_DEG)*dt*0.001;
    gyro_angle_yaw = gyro_angle_yaw + (gyro_raw_yaw*RAD_TO_DEG*dt*0.001);

    cf_pitch = (gain * gyro_angle_pitch) + (1.0-gain)*pitch;
    cf_roll = (gain * gyro_angle_roll) + (1.0-gain)*roll;

    Serial.print("cf_pitch:");
    Serial.println(cf_pitch);

    Serial.print("cf_roll:");
    Serial.println(cf_roll);

    Serial.print("gyro_angle_yaw:");
    Serial.println(gyro_angle_yaw);


  }
  // put your main code here, to run repeatedly:
if(cf_roll>50)
{
thruster.writeMicroseconds(1550);//esc struggles with lower throttle
}
else{
  thruster.writeMicroseconds(1500);
}

  last = now;
}



