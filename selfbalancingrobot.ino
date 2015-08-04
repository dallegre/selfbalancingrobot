//mpu6050 sketch for eventually doing the old inverted pendulum thing.
//i2c library
//added this comment to look at a change in a git repository.
#include<Wire.h>

#include "mpu.h"
#include "motors.h"
#include "calc.h"

int i = 0, data = 0;

//raw data variables
int16_t ax, ay, az, xangle, yangle;
int16_t gx, gy, gz;

//calculation variables
int angle, adjustment;
float coeffa = .08, coeffb = -.02;   //.08, -.02

mpu5060 mpu;
motors  motorcontrol;
calc    calculus;

void setup() {
  
  Serial.begin(38400);

  Serial.println("lasdkfj");

  motorcontrol.setup_motors();
  motorcontrol.go(0,0,0);
  
  mpu.setup_i2c();
  mpu.verify_i2c();
  mpu.initialize_chip();
  mpu.calibrate_gyro();

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

  //Serial.println();
  //Serial.println();
  //Serial.print("gy is: ");
  //Serial.println(gy);
  //Serial.print("az is: ");
  //Serial.println(az);
  //Serial.print("xangle is: ");
  //Serial.println(xangle + 500);

  angle = xangle;

  adjustment = (int)(coeffa*(float)angle + coeffb*(float)calculus.derivative(angle));

  //Serial.print("angle is: ");
  //Serial.println(angle);

  if(adjustment > 0)
    motorcontrol.go(1,1,adjustment/2);
  if(adjustment < 0)
    motorcontrol.go(0,1,abs(adjustment)/2);
  
  i = ~i;
  if(i){
    digitalWrite(ledPin, LOW);
    //motorcontrol.go(1,0,30);
  }else{
    digitalWrite(ledPin, HIGH);
    //motorcontrol.go(0,0,30);
  }

}

