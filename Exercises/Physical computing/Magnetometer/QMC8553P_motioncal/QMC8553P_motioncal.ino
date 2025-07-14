/*
  QMC5883P magnetometer calibration code for MotionCal
  Author: Ivan Novosel
  Date: 2025-07-13

  This code is intended for use with MotionCal software you can download from:
  https://www.pjrc.com/store/prop_shield.html
  It sends the QMC8553P magnetometer data to MotionCal software that generates the corrections
  for hard and soft iron sources.

  For details on magnetometer calibration read/watch this:
  https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
  
  The Digikey tutorial from the link uses MotionCal by PJRC and sensors and code from Adafruit:
  https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-motioncal
  But the QMC5883P is not supported in that example. This code just prints data to Serial in a format that
  MotionCal will understand. All the data is multiplied by 1000 so it fits nicely in the MotionCal window as
  it doesn't have scaling. The factor is 1/15 if applied to raw data (the code below uses the gauss function).
  When using the hard iron offset scale it back in your code, but leave the soft iron correction matrix as is.
*/


#include <Wire.h>

// default I2C address for QMC5883P
#define QMC5883P_ADDR 0x2C

// QMC5883P register addresses
#define CHIP_ID_REG 0x00
#define X_LSB_REG   0x01
#define X_MSB_REG   0x02
#define Y_LSB_REG   0x03
#define Y_MSB_REG   0x04
#define Z_LSB_REG   0x05
#define Z_MSB_REG   0x06
#define STATUS_REG  0x09
#define SAMPLE_MODE_REG 0x0A
#define CONFIG_REG  0x0B

// bit value masks for STATUS register
#define STATUS_DRDY 0x01
#define STATUS_OVFL 0x02

// bit value masks for sampling mode in SAMPLE_MODE_REG
#define SUSPEND     0x00
#define NORMAL      0x01
#define SINGLE      0x02
#define CONTINUOUS  0x03

// bit value masks for output data rate ODR in SAMPLE_MODE_REG
#define ODR_10_HZ   0x00
#define ODR_50_HZ   0x04
#define ODR_100_HZ  0x08
#define ODR_200_HZ  0x0C

// bit value masks for oversample ratio OSR1 in SAMPLE_MODE_REG
#define OSR8  0x00
#define OSR4  0x10
#define OSR2  0x20
#define OSR1  0x30

// bit value masks for downsample ratio OSR2 in SAMPLE_MODE_REG
#define DSR1  0x00
#define DSR2  0x40
#define DSR4  0x80
#define DSR8  0xC0

// bit value masks for SET/RESET mode in CONFIG_REG
#define SR_ON   0x00
#define S_ON    0x01
#define SR_OFF  0x02

// bit value masks for range mode in CONFIG_REG
#define RNG_30_GAUSS    0x00
#define RNG_12_GAUSS    0x04
#define RNG_8_GAUSS     0x08
#define RNG_2_GAUSS     0x0C

// bit value mask for self test mode in CONFIG_REG
#define SELF_TEST 0x40

// bit value mask for soft reset in CONFIG_REG
#define SOFT_RESET 0x80

// default values for initialization of the sensor
// doesn't seem to work in other modes or other sampling rates (behaviour doesn't match documentation)
void initQMC5883P(uint8_t mode=CONTINUOUS, // continuous mode
                  uint8_t data_rate=ODR_200_HZ, // output rate of 200 Hz
                  uint8_t oversample=OSR8, // maximum oversample
                  uint8_t downsample=DSR1,  // no downsample
                  uint8_t sr_mode=SR_ON, // set and reset both on
                  uint8_t range=RNG_2_GAUSS); // in most sensitive range

// sets all configuration settings, provide your own if you don't want the default above
void initQMC5883P(uint8_t mode, uint8_t data_rate, uint8_t oversample, uint8_t downsample, uint8_t sr_mode, uint8_t range) {
  // defines the direction of the reference system
  // mentioned application examples
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(0x29);
  Wire.write(0x06);
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(SAMPLE_MODE_REG);
  Wire.write(downsample|oversample|data_rate|mode);
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(0x00|sr_mode|range);
  Wire.endTransmission();
}

void reset(){
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(SOFT_RESET);
  Wire.endTransmission();
}

int get_config(uint8_t& regA, uint8_t& regB){
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(SAMPLE_MODE_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, 2);
  if (Wire.available() == 2){
    regA = Wire.read();
    regB = Wire.read();
    return 1;
  }
  return 0;
}

// check if there is data to read
bool data_ready(){
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(STATUS_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, 1);
  if (Wire.available() == 1){
    uint8_t status = Wire.read();
    return status & STATUS_DRDY;
  }
  return 0;
}

// read raw sensor data
void read_raw_mag(int16_t& x, int16_t& y, int16_t& z) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(X_LSB_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, 6);

  if (Wire.available() == 6) {
    uint8_t x_lsb = Wire.read();
    uint8_t x_msb = Wire.read();
    uint8_t y_lsb = Wire.read();
    uint8_t y_msb = Wire.read();
    uint8_t z_lsb = Wire.read();
    uint8_t z_msb = Wire.read();

    x = (x_msb << 8) | x_lsb;
    y = (y_msb << 8) | y_lsb;
    z = (z_msb << 8) | z_lsb;
  }
}

// default is the 2 Gauss range
void read_gauss_mag(float& x, float& y, float& z, uint8_t range=RNG_2_GAUSS);

// read sensor data and convert it to gauss units
void read_gauss_mag(float& x, float& y, float& z, uint8_t range) {
  int x_raw, y_raw, z_raw;
  float lsb_factor = 1;

  // this is different to documentation
  // seems that they set the maximum value at 30000
  switch(range){
    case RNG_2_GAUSS:
      lsb_factor = 32768/2;
      break;
    case RNG_8_GAUSS:
      lsb_factor = 32768/8;
      break;
    case RNG_12_GAUSS:
      lsb_factor = 32768/12;
      break;
    case RNG_30_GAUSS:
      lsb_factor = 32768/30;
      break;
  }

  read_raw_mag(x_raw,y_raw,z_raw);
  x = x_raw / lsb_factor;
  y = y_raw / lsb_factor;
  z = z_raw / lsb_factor;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  initQMC5883P();
}

void loop() {
  float mag_x, mag_y, mag_z;
  if (data_ready()){
    read_gauss_mag(mag_x, mag_y, mag_z);
  }
  
  // 'Raw' values to match expectation of MotionCal
  // first 3 digits are accelerometer data, next 3 digits are gyroscope data, and last 3 are magnetometer
  // if some data is 0 it is ignored, we only want to calibrate the magnetometer here
  Serial.print("Raw:");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");
  Serial.print(0); Serial.print(",");
  // the numbers are scaled down to be visible inside MotionCal
  Serial.print(int(mag_x*1000)); Serial.print(",");
  Serial.print(int(mag_y*1000)); Serial.print(",");
  Serial.println(int(mag_z*1000));

  delay(20);
}