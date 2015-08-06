//mpu6050 sketch for eventually doing the old inverted pendulum thing.
//i2c library
//added this comment to look at a change in a git repository.
//dur dur turpa derp.
#include<Wire.h>

#include "mpu.h"
#include "motors.h"
#include "calc.h"

int i = 0, data = 0;

//raw data variables
int16_t ax, ay, az, xangle, yangle;
int16_t gx, gy, gz;

//calculation variables
int angle, adjustment, calib;
float coeffa = .10, coeffb = -.1;   //.08, -.02

mpu6050 mpu;
motors  motorcontrol;
calc    calculus;

void setup() {
  
  Serial.begin(38400);

  motorcontrol.setup_motors();
  motorcontrol.go(0,0,0);
  
  mpu.setup_i2c();
  mpu.verify_i2c();
  mpu.initialize_chip();
  //mpu.calibrate_gyro();
  mpu.calibrate_accel();

}

void loop() {

  delay(10);
  
  mpu.get_gyro_rates();
  mpu.get_accel_values();
  mpu.get_accel_angles();
  gx = mpu.getx();
  gy = mpu.gety();        //I believe this is the axis you want
  gz = mpu.getz();
  ax = mpu.get_accelx();
  ay = mpu.get_accely();
  az = mpu.get_accelz();
  xangle = mpu.get_accel_xangle();
  yangle = mpu.get_accel_yangle();

  angle = xangle;

  adjustment = (int)(coeffa*(float)angle - coeffb*(float)calculus.derivative(angle));

  if(adjustment > 0)
    motorcontrol.go(1,1,adjustment/2);
  if(adjustment < 0)
    motorcontrol.go(0,1,abs(adjustment)/2);
  
  i = ~i;
  if(i){
    digitalWrite(ledPin, LOW);
  }else{
    digitalWrite(ledPin, HIGH);
  }

}

