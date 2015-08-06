#include "mpu_defines.h"

const int ledPin =  13;

class mpu6050{
  public:
    int  data;
    int  gyro_xout_h, gyro_xout_l, gyro_yout_h, gyro_yout_l, gyro_zout_h, gyro_zout_l,
      gyro_xout, gyro_yout, gyro_zout, gyro_xrate, gyro_yrate, gyro_zrate,
      gyro_xoffset, gyro_yoffset, gyro_zoffset, gyro_xsum, gyro_ysum, gyro_zsum,
      accel_xout_h, accel_xout_l, accel_yout_h, accel_yout_l, accel_zout_h, accel_zout_l,
      accel_xout, accel_yout, accel_zout, accel_xangle, accel_yangle, accel_average;
    int  gyro_xsensitivity = 20, gyro_ysensitivity = 20, gyro_zsensitivity = 20;
    void setup_i2c(void);
    void verify_i2c(void);
    int  read_i2c(int address);
    void write_i2c(int address, int data);
    void initialize_chip(void);
    void get_gyro_rates(void);
    void calibrate_gyro(void);
    void get_accel_values(void);
    void get_accel_angles(void);
    int  getx(void);
    int  gety(void);
    int  getz(void);
    int  get_accelx(void);
    int  get_accely(void);
    int  get_accelz(void);
    int  get_accel_xangle(void);
    int  get_accel_yangle(void);
    void calibrate_accel(void);
};

void mpu6050::setup_i2c(void){

  pinMode(ledPin,  OUTPUT);
  Wire.begin(MPU6050_ADDRESS);
  delay(1000);

}

int mpu6050::read_i2c(int address){

  Wire.beginTransmission(0xd2>>1);
  Wire.send(address);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDRESS>>1,1);
  while(Wire.available())
    data = Wire.receive();
  Wire.endTransmission();
  return data;

}

void mpu6050::write_i2c(int address, int data){
  
  Wire.beginTransmission(0xd2>>1);
  Wire.send(address);
  Wire.send(data);
  Wire.endTransmission();

}

void mpu6050::verify_i2c(void){
  
  data = this->read_i2c(MPU6050_RA_WHO_AM_I);
  
  if(data == 0x68){
    Serial.println("the i2c is setup OK");
  }else{
    Serial.println("the i2c is not setup OK");
  }

}

void mpu6050::initialize_chip(void){
  //Sets sample rate to 8000/1+7 = 1000Hz
  this->write_i2c(MPU6050_RA_SMPLRT_DIV, 0x07);
  //Disable FSync, 20Hz DLPF
  this->write_i2c(MPU6050_RA_CONFIG, 0x04);
  //Disable gyro self tests, scale of 500 degrees/s
  this->write_i2c(MPU6050_RA_GYRO_CONFIG, 0b00001000);
  //Disable accel self tests, scale of +-2g, no DHPF
  this->write_i2c(MPU6050_RA_ACCEL_CONFIG, 0x00);
  //Freefall threshold of |0mg|
  //can't find this register in the register map.
  //this->write_i2c(MPU6050_RA_FF_THR, 0x00);
  //Freefall duration limit of 0
  //can't find this register in the register map.
  //this->write_i2c(MPU6050_RA_FF_DUR, 0x00);
  //Motion threshold of 0mg
  this->write_i2c(MPU6050_RA_MOT_THR, 0x00);
  //Motion duration of 0s
  //can't find this register in the register map.
  //this->write_i2c(MPU6050_RA_MOT_DUR, 0x00);
  //Zero motion threshold
  //can't find this register in the register map.
  //this->write_i2c(MPU6050_RA_ZRMOT_THR, 0x00);
  //Zero motion duration threshold
  //can't find this register in the register map.
  //this->write_i2c(MPU6050_RA_ZRMOT_DUR, 0x00);
  //Disable sensor output to FIFO buffer
  this->write_i2c(MPU6050_RA_FIFO_EN, 0x00);
  
  //AUX I2C setup
  //Sets AUX I2C to single master control, plus other config
  this->write_i2c(MPU6050_RA_I2C_MST_CTRL, 0x00);
  //Setup AUX I2C slaves
  this->write_i2c(MPU6050_RA_I2C_SLV0_ADDR, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV0_REG, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV0_CTRL, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV1_ADDR, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV1_REG, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV1_CTRL, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV2_ADDR, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV2_REG, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV2_CTRL, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV3_ADDR, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV3_REG, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV3_CTRL, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV4_ADDR, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV4_REG, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV4_DO, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV4_CTRL, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV4_DI, 0x00);
  
  //Setup INT pin and AUX I2C pass through.  make int pin open drain.
  this->write_i2c(MPU6050_RA_INT_PIN_CFG, 0x40);
  //disable all interrupts
  this->write_i2c(MPU6050_RA_INT_ENABLE, 0x00);
  
  //Slave out, dont care
  this->write_i2c(MPU6050_RA_I2C_SLV0_DO, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV1_DO, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV2_DO, 0x00);
  this->write_i2c(MPU6050_RA_I2C_SLV3_DO, 0x00);
  //More slave config
  this->write_i2c(MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
  //Reset sensor signal paths
  this->write_i2c(MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
  //Motion detection control
  this->write_i2c(MPU6050_RA_MOT_DETECT_CTRL, 0x00);
  //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
  this->write_i2c(MPU6050_RA_USER_CTRL, 0x00);
  //Sets clock source to gyro reference w/ PLL
  this->write_i2c(MPU6050_RA_PWR_MGMT_1, 0b00000010);
  //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
  this->write_i2c(MPU6050_RA_PWR_MGMT_2, 0x00);
  
  //Data transfer to and from the FIFO buffer
  this->write_i2c(MPU6050_RA_FIFO_R_W, 0x00);
  
  Serial.println("\nMPU6050 Setup Complete");
  Serial.println();
}

void mpu6050::get_gyro_rates(void){
  
  gyro_xout_h = this->read_i2c(MPU6050_RA_GYRO_XOUT_H);
  gyro_xout_l = this->read_i2c(MPU6050_RA_GYRO_XOUT_L);
  gyro_yout_h = this->read_i2c(MPU6050_RA_GYRO_YOUT_H);
  gyro_yout_l = this->read_i2c(MPU6050_RA_GYRO_YOUT_L);
  gyro_zout_h = this->read_i2c(MPU6050_RA_GYRO_ZOUT_H);
  gyro_zout_l = this->read_i2c(MPU6050_RA_GYRO_ZOUT_L);
  
  gyro_xout = ((gyro_xout_h<<8)|gyro_xout_l) - gyro_xoffset;   //will want to add calibration offsets later.
  gyro_yout = ((gyro_yout_h<<8)|gyro_yout_l) - gyro_yoffset;
  gyro_zout = ((gyro_zout_h<<8)|gyro_zout_l) - gyro_zoffset;

  //convert from unsigned to signed
  int gyro_xout_s = (short)gyro_xout;
  int gyro_yout_s = (short)gyro_yout;
  int gyro_zout_s = (short)gyro_zout;
  
  gyro_xrate = (float)gyro_xout_s/gyro_xsensitivity;  //not sure what these need to be set at
  gyro_yrate = (float)gyro_yout_s/gyro_ysensitivity;
  gyro_zrate = (float)gyro_zout_s/gyro_zsensitivity;

  //Serial.print("gyrosope x out is: ");
  //Serial.println(gyro_xrate);
  //Serial.print("gyrosope y out is: ");
  //Serial.println(gyro_yrate);
  //Serial.print("gyrosope z out is: ");
  //Serial.println(gyro_zrate);
  //Serial.println();
  
}

void mpu6050::calibrate_gyro(void){
  
  Serial.println("Calibrating gyroscope");
  Serial.println();
  for(int x = 0; x < 500; x++){
    gyro_xout_h = this->read_i2c(MPU6050_RA_GYRO_XOUT_H);
    gyro_xout_l = this->read_i2c(MPU6050_RA_GYRO_XOUT_L);
    gyro_yout_h = this->read_i2c(MPU6050_RA_GYRO_YOUT_H);
    gyro_yout_l = this->read_i2c(MPU6050_RA_GYRO_YOUT_L);
    gyro_zout_h = this->read_i2c(MPU6050_RA_GYRO_ZOUT_H);
    gyro_zout_l = this->read_i2c(MPU6050_RA_GYRO_ZOUT_L);
    gyro_xsum += ((gyro_xout_h<<8)|gyro_xout_l);
    if( ((gyro_yout_h<<8)|gyro_yout_l) < 250)
      gyro_ysum += ((gyro_yout_h<<8)|gyro_yout_l);
    gyro_zsum += ((gyro_zout_h<<8)|gyro_zout_l);
    delay(1);
  }

  gyro_xoffset = (float)gyro_xsum/500;
  gyro_yoffset = (float)gyro_ysum/500;
  gyro_zoffset = (float)gyro_zsum/500;

  Serial.println("Done calibrating");
  Serial.print("X offset is: ");
  Serial.println((short)gyro_xoffset);
  Serial.print("Y offset is: ");
  Serial.println((short)gyro_yoffset);
  Serial.print("Z offset is: ");
  Serial.println((short)gyro_zoffset);

  delay(1000);

}

int mpu6050::getx(void){
  return gyro_xrate;
}

int mpu6050::gety(void){
  return gyro_xrate;
}

int mpu6050::getz(void){
  return gyro_xrate;
}

void mpu6050::get_accel_values(){
  
  accel_xout_h = this->read_i2c(MPU6050_RA_ACCEL_XOUT_H);
  accel_xout_l = this->read_i2c(MPU6050_RA_ACCEL_XOUT_L);
  accel_yout_h = this->read_i2c(MPU6050_RA_ACCEL_YOUT_H);
  accel_yout_l = this->read_i2c(MPU6050_RA_ACCEL_YOUT_L);
  accel_zout_h = this->read_i2c(MPU6050_RA_ACCEL_ZOUT_H);
  accel_zout_l = this->read_i2c(MPU6050_RA_ACCEL_ZOUT_L);
 
  accel_xout = (short)((accel_xout_h<<8)|accel_xout_l);
  accel_yout = (short)((accel_yout_h<<8)|accel_yout_l);
  accel_zout = (short)((accel_zout_h<<8)|accel_zout_l);  
  
} 

void mpu6050::calibrate_accel(void){
  
  Serial.println();
  Serial.println("Calibrating accelerometer");
  
  int accel_total = 0;
  for(int x = 0; x < 500; x++){
    this->get_accel_values();
    this->get_accel_angles();
    accel_total += accel_xangle;
  }  
  accel_average = (float)accel_total/500;

  Serial.println("Done calibrating accelerometer");

}
 
//Converts the already acquired accelerometer data into 3D euler angles
void mpu6050::get_accel_angles(void){

  //the thing is oriented funny, so substitute these accordingly.
  accel_xangle = 100*57.295*atan((float)accel_zout/ sqrt(pow((float)accel_yout,2)+pow((float)accel_zout,2))) - accel_average;   //use calibration here
  accel_yangle = 100*57.295*atan((float)-accel_yout/ sqrt(pow((float)accel_zout,2)+pow((float)accel_yout,2)));  

} 

int mpu6050::get_accelx(void){
  return accel_xout;
}


int mpu6050::get_accely(void){
  return accel_yout;
}


int mpu6050::get_accelz(void){
  return accel_zout;
}

int mpu6050::get_accel_xangle(void){
  return accel_xangle;
}


int mpu6050::get_accel_yangle(void){
  return accel_yangle;
}
