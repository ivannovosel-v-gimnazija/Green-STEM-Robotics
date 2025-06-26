/*
	MPU6050 DMP Polling example
	2025-06-26
	by Ivan Novosel for Green STEM project
	based on code from FRANKWIN10\Frank and Jeff Rowberg

	This is a code example shows how to read DMP fused sensor data from an MPU6050 IMU.
	It will start the DMP, use it to get quaternion data and then export that into
	yaw, pitch and roll angles. It will then print those values to Serial in a way that
	is readable in both Serial monitor and Serial plotter of Arduino IDE.

	Most of the code is from:
	https://www.fpaynter.com/2019/10/polling-vs-interrupt-with-mpu6050-gy-521-and-arduino/
	by FRANKWIN10\Frank (2019-05-10)
	Which is itself adapted from:
	https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6_using_DMP_V6.12/MPU6050_DMP6_using_DMP_V6.12.ino
	by Jeff Rowberg (last update 2019-07-10)

	Why this code instead of exsisting examples?
	The original MPU6050 DMP example from I2Cdev library uses interrupts, which for me break the Serial prints.
	The program just hangs on the first Serial.print in the code.
	Franks version uses polling (no interrupts), but doesn't set the Wire frequency correctly.
	Because of that the DMP doesn't set up correctly and won't start.

	This example adds the code to set up the Wire from I2Cdev DMP example to Franks version so it works correctly.
	Also I cleaned up some of the code (no unused variables), and avoided using an external library for the timer.
	Also this version prints all YPR angles, not just yaw angle.
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
// you can switch to "MPU6050_6Axis_MotionApps20.h"
//it is an older version of MotionApp but is a smaller library, takes less space in Arduino memory
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for most boards)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//extra stuff
int IMU_CHECK_INTERVAL_MSEC = 100;
unsigned long lastIMUCheck;
int global_fifo_count = 0; //made global so can monitor from outside GetIMUHeadingDeg() fcn

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif

	Serial.begin(38400);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately

	// initialize device
	Serial.println(F("Initializing MPU6050..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.PrintActiveOffsets();

		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);


		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
	
	lastIMUCheck = 0; //this manages 'other program stuff' cycle
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
	// if programming failed, don't try to do anything
	if (!dmpReady) return;
 
	if (mpu.dmpPacketAvailable())
	{
		getYPR(); //retreive the most current yaw, pitch and roll data from IMU
	}

	unsigned long currentIMUcheck = millis();
	// calculate readable YPR values in degrees
	float yaw = ypr[0] * 180 / M_PI;
	float pitch = ypr[1] * 180 / M_PI;
	float roll = ypr[2] * 180 / M_PI;
 
	//other program stuff block - executes every IMU_CHECK_INTERVAL_MSEC Msec
	//for this test program, there's nothing here except diagnostics printouts
	if (currentIMUcheck - lastIMUCheck >= IMU_CHECK_INTERVAL_MSEC)
	{
		lastIMUCheck = currentIMUcheck;
		Serial.print("Roll:");
		Serial.print(roll);
		Serial.print(",");
		Serial.print("Pitch:");
		Serial.print(pitch);
		Serial.print(",");
		Serial.print("Yaw:");
		Serial.println(yaw);
		if (global_fifo_count != 0)
		{
			Serial.print("FIFO Reset!");
			mpu.resetFIFO();
		}
	}
}
 
void getYPR()
{
	// At least one data packet is available
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();// get current FIFO count
 
	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
 
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
	{
		// read all available packets from FIFO
		while (fifoCount >= packetSize) // Lets catch up to NOW, in case someone is using the dreaded delay()!
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;
		}
		global_fifo_count = mpu.getFIFOCount(); //should be zero here
 
		// calculate yaw pitch and roll
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	}
}
