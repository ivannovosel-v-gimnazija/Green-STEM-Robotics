/*
	Digital compass using MPU6050 and QMC8553P
	2025-07-13
	by Ivan Novosel for Green STEM project
	
	This code combines measurements of MPU6050 and QMC8533P to calculate heading, 0° = North).
	MPU6050 is used to get the down direction, while east and north are calculated by applying
	vector cross product, explained here:
	https://www.youtube.com/watch?v=0rlvvYgmTvI&t=26s

	The code also has calibration for hard and soft iron sources. If you dont' want to use that
	part set all coefficients of hard_cal and soft_cal to 0. For calibration procedure please use
	this code:
	https://github.com/ivannovosel-v-gimnazija/Green-STEM-Robotics/blob/main/Exercises/Physical%20computing/Magnetometer/QMC8553P_motioncal/QMC8553P_motioncal.ino
	and follow these instructions, just run the code above instead of imucal in step 2:
	https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-motioncal
	
	For more detailed explanation of code check the previous exercises:
	For MPU6050
	https://github.com/ivannovosel-v-gimnazija/Green-STEM-Robotics/blob/main/Exercises/Physical%20computing/Measuring%20angles%20with%20an%20IMU/mpu6050_polling/mpu6050_polling.ino
	For QMC8553P
	https://github.com/ivannovosel-v-gimnazija/Green-STEM-Robotics/blob/main/Exercises/Physical%20computing/Magnetometer/QMC8553P_magnetometer/QMC8553P_magnetometer.ino
*/

// includes for MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// includes and defins for QMC5883P
#define QMC5883P_ADDR 0x2C
#define CHIP_ID_REG 0x00
#define X_LSB_REG   0x01
#define STATUS_REG  0x09
#define SAMPLE_MODE_REG 0x0A
#define CONFIG_REG  0x0B
#define STATUS_DRDY 0x01
#define CONTINUOUS  0x03
#define ODR_10_HZ   0x00
#define ODR_50_HZ   0x04
#define ODR_100_HZ  0x08
#define ODR_200_HZ  0x0C
#define OSR8  0x00
#define OSR4  0x10
#define OSR2  0x20
#define OSR1  0x30
#define DSR1  0x00
#define DSR2  0x40
#define DSR4  0x80
#define DSR8  0xC0
#define SR_ON   0x00
#define S_ON    0x01
#define SR_OFF  0x02
#define RNG_30_GAUSS    0x00
#define RNG_12_GAUSS    0x04
#define RNG_8_GAUSS     0x08
#define RNG_2_GAUSS     0x0C
#define SOFT_RESET 0x80

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
// these are defined here as global variables not because they must be global but because
// it can happen  that the sensor isn't ready to read data, and that will ocassionally leave
// fake zeroes in our data. this way if there is no data to read the old value will just persist
Quaternion q;				// [w, x, y, z]         quaternion container
VectorFloat mag;		// [x, y, z]						magnetic filed intensity
VectorFloat down;		// [x, y, z]            gravity vector
VectorFloat east;		// [x, y, z]            east vector
VectorFloat north;	// [x, y, z]            north vector

// hard iron calibration
// magnetic offset from MotionCal scaled back
const float hard_cal[3] = {18.74/1000, -10.38/1000, -7.82/1000};

//soft iron calibration
// magnetic mapping from MotionCal in matrix form
const float soft_cal[3][3] = {{0.970, 0.011, 0.137},
															{0.011,1.034,-0.027},
															{0.137,0.027,1.017}};

// declination for Zagreb, Croatia - use https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml for your location
const float mag_declination = 4.9;
float heading = 0;

// timer for the slover code, ex. prints
const int TIMER = 100;
unsigned long previousTime;
int global_fifo_count = 0;

// calculates Down vector, normalised free fall acceleration
void getDown()
{
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
	{
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
	}
	else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
	{
		while (fifoCount >= packetSize)
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;
		}
		global_fifo_count = mpu.getFIFOCount();
 
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&down, &q);
		down.normalize(); // normalize the down vector
	}
}

// default values for initialization of the QMC5883P sensor
void initQMC5883P(uint8_t mode=CONTINUOUS, // continuous mode
                  uint8_t data_rate=ODR_200_HZ, // output rate of 200 Hz
                  uint8_t oversample=OSR8, // maximum oversample
                  uint8_t downsample=DSR1,  // no downsample
                  uint8_t sr_mode=SR_ON, // set and reset both on
                  uint8_t range=RNG_2_GAUSS); // in most sensitive range

// sets all configuration settings, provide your own if you don't want the default above
void initQMC5883P(uint8_t mode, uint8_t data_rate, uint8_t oversample, uint8_t downsample, uint8_t sr_mode, uint8_t range) {
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

// check if there is data to read from QMC5883P
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

// read raw QMC5883P magnetometer data
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
	float xh, yh, zh;
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
	// convert to gauss and apply hard iron calibration
  xh = (x_raw / lsb_factor) - hard_cal[0];
  yh = (y_raw / lsb_factor) - hard_cal[1];
  zh = (z_raw / lsb_factor) - hard_cal[2];
	// apply calibration for soft iron sources
	x = soft_cal[0][0]*xh + soft_cal[0][1]*yh + soft_cal[0][2]*zh;
	y = soft_cal[1][0]*xh + soft_cal[1][1]*yh + soft_cal[1][2]*zh;
	z = soft_cal[2][0]*xh + soft_cal[2][1]*yh + soft_cal[2][2]*zh;
}

void setup()
{
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif

	Serial.begin(38400);
	while (!Serial);

	// initialize QMC8553P
	initQMC5883P();
	// initialize MPU6050 and its DMP
	Serial.println(F("Initializing MPU6050..."));
	mpu.initialize();
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	if (devStatus == 0)
	{
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.PrintActiveOffsets();
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
	previousTime = 0;
}

void loop()
{
	if(!dmpReady) return;
	// read MPU6050
	if (mpu.dmpPacketAvailable())
	{
		getDown();
	}
	// read magnetometer
	if (data_ready()){
    read_gauss_mag(mag.x, mag.y, mag.z);
  }
	mag.normalize();
	// east = mag X down
	east.x = mag.y*down.z - mag.z*down.y;
	east.y = mag.z*down.x - mag.x*down.z;
	east.z = mag.x*down.y - mag.y*down.x;
	east.normalize();
	// north = down x east
	north.x = down.y*east.z - down.z*east.y;
	north.y = down.z*east.x - down.x*east.y;
	north.z = down.x*east.y - down.y*east.x;

	// heading calculation
	heading = atan2(east.x, north.x)*180.0/PI + mag_declination;
	if (heading<0) heading += 360;

	unsigned long currentTime = millis();
	if (currentTime - previousTime >= TIMER)
	{
		previousTime = currentTime;
		Serial.print("Heading:");
		Serial.println(heading);

		if (global_fifo_count != 0)
		{
			Serial.print("FIFO Reset!");
			mpu.resetFIFO();
		}
	}
}