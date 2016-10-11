/*
	Main program which controls the quadrocopter

 Name:		Quadrocopter.ino
 Author:	Marc Berchtold & Patrick Schenker
*/

//Wire: I2C communication - https://www.arduino.cc/en/Reference/Wire
//I2Cdev: Provides simple and intuitive interfaces to I2C devices (Depends on Wire.h) - https://github.com/jrowberg/i2cdevlib
//PID: PID Controller - https://github.com/br3ttb/Arduino-PID-Library/
//PinChangeInt: Implements pin change interrupts - https://github.com/GreyGnome/PinChangeInt
//Servo: Implements easy PWM signals over pins - https://www.arduino.cc/en/Reference/Servo
//helper_3dmath: Implements support for multiple 3D calculations (Quaternions, Vectors, etc.) - https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//MPU6050_6Axis_MotionApps20: Implements support for the DMP processor in the MPU 9150 chip (Depends on helper_3dmath.h) - https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

#include <I2Cdev.h>
#include <Wire.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <helper_3dmath.h>

#define POWER_MINIMUM 0

#define MOTOR0_PIN 6
#define MOTOR1_PIN 9
#define MOTOR2_PIN 10
#define MOTOR3_PIN 11

#define MOTOR_ZERO_VALUE 1000
#define MOTOR_MAX_VALUE 2000

#define RECEIVER_THROTTLE 7 //On Receiver Channel 3
#define RECEIVER_PITCH 3 //On Receiver Channel 2
#define RECEIVER_ROLL 4 //On Receiver Channel 4
#define RECEIVER_YAW 5 //On Receiver Channel 1


//Motor powers/////////////////////
int motor0Power = 0; //Front
int motor1Power = 0; //Right
int motor2Power = 0; //Back
int motor3Power = 0; //Left
///////////////////////////////////

//Receiver Signals/////////////////
#define POWERRECEIVERMIN  1112
#define POWERSAFESHUTDOWN 1250
#define POWERRECEIVERMAX  1936
#define POWERRECEIVERMIDDLE  1516

#define ROLLRECEIVERMIN  1108
#define ROLLRECEIVERMAX  1936
#define ROLLRECEIVERMIDDLE 1520
#define ROLLVALUEMIN  -30
#define ROLLVALUEMAX  30

#define PITCHRECEIVERMIN  1116
#define PITCHRECEIVERMAX  1952
#define PITCHRECEIVERMIDDLE 1552
#define PITCHVALUEMIN  -30
#define PITCHVALUEMAX  30

#define YAWRECEIVERMIN  1108
#define YAWRECEIVERMAX  1936
#define YAWRECEIVERMIDDLE 1524
#define YAWVALUEMIN  -30
#define YAWVALUEMAX  30

int power = POWER_MINIMUM;
//Volatile forces compiler to load variables from RAM and not from storage registry --> Makes changes from other threads secure (for example interrupts)
volatile int receiverRawValue[8]; //Power, Pitch, Roll, Yaw

//Holds last pin that has risen / fallen.
uint8_t lastInterruptedPin;
//Holds the micros() value of the moment the rising interrupt has been triggered for each pin (Throttle, Pitch, Roll, Yaw)
volatile int pwmStartMicroseconds[8];
//////////////////////////////////

//Angles of the quadrocopter//////
float angle_X = 0.0f; //Pitch
float angle_Y = 0.0f; //Roll
float angle_Z = 0.0f; //Yaw
float yawPitchRoll[3];
//////////////////////////////////

//PID variables///////////////////
//Same pid values for Pitch and Roll as it is a quadrocopter (same on all four sides)
//Experimental pid values need more fine tuning
#define pidPitchP 0.250
#define pidPitchI 0.950
#define pidPitchD 0.011
#define pidPitchMax 200.0
#define pidPitchMin -200.0
double pidPitch_In = 0;
double pidPitch_Out = 0;
double pidPitch_Setpoint = 0;
//Create the new PID controller by passing the input, setpoint and variable where the output should be saved. Also pass along the 3 PID values (P,I,D) and the direction (Reversed or direct)
//& before function name forces call-by-reference instead of call-by-value
PID pitchPID(&pidPitch_In, &pidPitch_Out, &pidPitch_Setpoint, 5.0, 0.0, 0.0, REVERSE);

#define pidRollP 0.250
#define pidRollI 0.950
#define pidRollD 0.011
#define pidRollMax 200.0
#define pidRollMin -200.0
double pidRoll_In = 0;
double pidRoll_Out = 0;
double pidRoll_Setpoint = 0;
PID rollPID(&pidRoll_In, &pidRoll_Out, &pidRoll_Setpoint, 5.0, 0.0, 0.0, REVERSE);

#define pidYawP 0.680
#define pidYawI 0.500
#define pidYawD 0.0001
#define pidYawMax 100.0
#define pidYawMin -100.0
//TODO: Find perfect min max values
double pidYaw_In = 0;
double pidYaw_Out = 0;
double pidYaw_Setpoint = 0;
PID yawPID(&pidYaw_In, &pidYaw_Out, &pidYaw_Setpoint, 1.0, 0.0, 0.0, DIRECT);
/////////////////////////////////

//MPU////////////////////////////
MPU6050 mpu;

bool dmpIsReady = false; //Holds if DMP Processor is ready and can process information
uint8_t mpuInterruptStatus; //Holds the raw value the Interrupt pin is currently on
volatile bool mpuInterrupt = false; //Holds if MPU interrupt pin is high or low
uint8_t mpuDeviceStatus;	//Returned status after each device operation (if not 0 = error)
uint8_t mpuPacketSize;		//How big the packet size of the DMP processor is (default 42bytes)
uint16_t mpuFIFOCount;		//How many bytes are currently stored in the FIFO of the DMP
							//FIFO: A 1024byte buffer inside the MPU which holds raw sensor values. As soon as some data
							//is placed inside the FIFO buffer the interrupt pin (INT) on the MPU is triggered.
uint8_t mpuFIFOData[64];	//Stores the last 64 values

Quaternion mpuQuaternion;				//Holds the quaternion data from the DMP
VectorInt16 mpuRawAcceleration;			//Holds raw data from the acceleration sensor
VectorInt16 mpuGravityFreeAcceleration;	//Holds data from acceleration sensor minus the inpact of the worlds gravity on the measurement
VectorInt16 mpuWorldAcceleration;		//Holds the acceleration data in the frame of the world.
VectorFloat mpuGravity;					//Holds the gravity vectors
float mpuEuler[3];						//Holds the euler angles of the quadrcopter
/////////////////////////////////

//Servos/////////////////////////
Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;
/////////////////////////////////


//Time when the Loop ended
//Unsigned --> No negativ values --> If value bigger than 2,147,483,647 it will fall back to 0 and continue instead of flipping to most negativ number (-2,147,483,648)
unsigned long endLoopTime = 0;
//Time when the Loop started
unsigned long startLoopTime = 0;
//Holds how many times the loop funktion has been started, gets reset to 0 after it reaches 10
unsigned int loopCycleCounter = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(112500);

	//Enable Input mode for all receiver pins
	pinMode(RECEIVER_THROTTLE, INPUT);
	pinMode(RECEIVER_PITCH, INPUT);
	pinMode(RECEIVER_ROLL, INPUT);
	pinMode(RECEIVER_YAW, INPUT);

	//Enable the internal pullup resistor (so that the attached Receiver has to pull the pins low)
	digitalWrite(RECEIVER_THROTTLE, HIGH);
	digitalWrite(RECEIVER_PITCH, HIGH);
	digitalWrite(RECEIVER_ROLL, HIGH);
	digitalWrite(RECEIVER_YAW, HIGH);

	//Attach rising pin Interrupts to all receiver pins
	//& before function name forces call-by-reference instead of call-by-value
	PCintPort::attachInterrupt(RECEIVER_THROTTLE, &pinIsRising, RISING);
	PCintPort::attachInterrupt(RECEIVER_PITCH, &pinIsRising, RISING);
	PCintPort::attachInterrupt(RECEIVER_ROLL, &pinIsRising, RISING);
	PCintPort::attachInterrupt(RECEIVER_YAW, &pinIsRising, RISING);


	//Start the I2C communication with the MPU
	Wire.begin();
	//Initialize the MPU
	mpu.initialize();
	//Wait for 5 seconds before initializing / calibrating the MPU so the sensors can settle down
	delay(5000);
	//Initialize the DMP processor and save the return code into the device status variable
	mpuDeviceStatus = mpu.dmpInitialize();
	//If the DMP in the MPU initiliazed correctly proceed (e.g if no errors detected)
	if (mpuDeviceStatus == 0) {
		//Enable the DMP co-processor for usage
		mpu.setDMPEnabled(true);
		//Attach the interrupt pin 0 (pin D2) and set it to set the mpuInterrupt var to true when the pin goes high
		attachInterrupt(0, mpuInterruption, RISING);
		//Get the current interrupt status of the MPU
		mpuInterruptStatus = mpu.getIntStatus();
		//Get the current packet size of the FIFO buffer
		mpuPacketSize = mpu.dmpGetFIFOPacketSize();

		//The DMP co-processor is all configured now and ready, let's set the flag to true
		dmpIsReady = true;
	}


	//Initialize PID Controllers
	//First set the Output Limits the PID controller can use. These are hard limits and the PID controller won't go over that
	pitchPID.SetOutputLimits(pidPitchMin, pidPitchMax);
	//Automatic mode turns the PID controller on (MANUAL = off, default is MANUAL on creation)
	pitchPID.SetMode(AUTOMATIC);
	//How often in miliseconds the PID calculations will be performed (Default = 200ms)
	pitchPID.SetSampleTime(10);

	rollPID.SetOutputLimits(pidRollMin, pidRollMax);
	rollPID.SetMode(AUTOMATIC);
	rollPID.SetSampleTime(10);

	yawPID.SetOutputLimits(pidYawMin, pidYawMax);
	yawPID.SetMode(AUTOMATIC);
	yawPID.SetSampleTime(10);
	//TODO: Maybe we can decrease sample time for yaw? Doesn't need to be as much stabilized as pitch & roll



	//Basic setup is done now.

	//Initialize motors
	motor0.attach(MOTOR0_PIN);
	motor1.attach(MOTOR1_PIN);
	motor2.attach(MOTOR2_PIN);
	motor3.attach(MOTOR3_PIN);
	//As a last step let's arm the ESCs and Motors
	motor0.writeMicroseconds(MOTOR_ZERO_VALUE);
	motor1.writeMicroseconds(MOTOR_ZERO_VALUE);
	motor2.writeMicroseconds(MOTOR_ZERO_VALUE);
	motor3.writeMicroseconds(MOTOR_ZERO_VALUE);
}

// the loop function runs over and over again until power down or reset
void loop() {
	//Save the microseconds since alive when the loop started
	startLoopTime = micros();

	//If the DMP co-processor isn't ready yet we can't do anything so let's skip this iteration of the loop
	if (dmpIsReady == false)
		return;

	//While the MPU didn't trigger the interrupt we don't have to do calculations on the MPU data so we can do other stuff.
	//We also check if the the FIFO buffer contains enough data for a valid packet
	while (mpuInterrupt != true && mpuFIFOCount < mpuPacketSize)
	{
		//Set the power level based on the pwm inputs mapped to the max & min PWM values of our escs
		power = map(receiverRawValue[7], POWERRECEIVERMIN, POWERRECEIVERMAX, MOTOR_ZERO_VALUE, MOTOR_MAX_VALUE);

		//Update our PID setpoints
		//Check we are at the neutral / middle level on the remote
		//Else set the PID setpoint value of the channel to the remaped value
		
		//Pitch
		if (receiverRawValue[3] > PITCHRECEIVERMIDDLE - 20 && receiverRawValue[3] < PITCHRECEIVERMIDDLE + 20)
			pidPitch_Setpoint = 0;
		else
			pidPitch_Setpoint = map(receiverRawValue[3], PITCHRECEIVERMIN, PITCHRECEIVERMAX, PITCHVALUEMIN, PITCHVALUEMAX);

		//Let's check again if the MPU didn't trigger the interrupt. If it did we have to break so we can process the data as soon as possible (so we don't get FIFO overflows)
		if (!(mpuInterrupt != true && mpuFIFOCount < mpuPacketSize))
			break;
		//Roll
		if (receiverRawValue[4] > ROLLRECEIVERMIDDLE - 20 && receiverRawValue[4] < ROLLRECEIVERMIDDLE + 20)
			pidRoll_Setpoint = 0;
		else
			pidRoll_Setpoint = map(receiverRawValue[4], ROLLRECEIVERMIN, ROLLRECEIVERMAX, ROLLVALUEMIN, ROLLVALUEMAX);
		if (!(mpuInterrupt != true && mpuFIFOCount < mpuPacketSize))
			break;
		//Yaw
		if (receiverRawValue[5] > YAWRECEIVERMIDDLE - 20 && receiverRawValue[5] < YAWRECEIVERMIDDLE + 20)
			pidYaw_Setpoint = 0;
		else
			pidYaw_Setpoint = map(receiverRawValue[5], YAWRECEIVERMIN, YAWRECEIVERMAX, YAWVALUEMIN, YAWVALUEMAX);



		//Update the PID in variables to the measured angles from the MPU's DMP
		pidPitch_In = angle_X;
		pidRoll_In = angle_Y;
		pidYaw_In = angle_Z;

		//Next step is quite time consuming so let's check before if the MPU didn't trigger an interrupt yet
		if (!(mpuInterrupt != true && mpuFIFOCount < mpuPacketSize))
			break;
		//Let the PID controller compute the newest values
		pitchPID.Compute();
		rollPID.Compute();
		yawPID.Compute();

		//Set the motor variables based on the calculated values
		motor0Power = power + pidPitch_Out + pidYaw_Out;
		motor1Power = power + pidRoll_Out - pidYaw_Out;
		motor2Power = power - pidPitch_Out + pidYaw_Out;
		motor3Power = power - pidRoll_Out - pidYaw_Out;

		//If the throttle / power is lower then the safe shutdown value turn them off (this is here to prevent motors to accidentaly turning on due to noise in the receiver end (2.4 ghz spectrum))
		//Also prevents motors to accidentaly turn when the quadrocopter is turned on its side and the pid controllers therefore calculate a very high correction Value
		if (power < POWERSAFESHUTDOWN) {
			motor0Power = MOTOR_ZERO_VALUE;
			motor1Power = MOTOR_ZERO_VALUE;
			motor2Power = MOTOR_ZERO_VALUE;
			motor3Power = MOTOR_ZERO_VALUE;
		}

		//Write the motor power values to the ESCs
		motor0.writeMicroseconds(motor0Power);
		motor1.writeMicroseconds(motor1Power);
		motor2.writeMicroseconds(motor2Power);
		motor3.writeMicroseconds(motor3Power);
	}

	//Since we got till here the interrupt of the MPU has obviously been triggered (else we would still be in the while loop) so now we can start processing
	//the data. We can reset the variables and get the current interrupt status.
	mpuInterrupt = false;
	mpuInterruptStatus = mpu.getIntStatus();

	//Get the current amount of data in the FIFO buffer
	mpuFIFOCount = mpu.getFIFOCount();

	//Check if the FIFO is overflowing with too much data (max is 1024 bytes)
	//Either the device status reflects this (decimal 16) or when the amount of data is exactly 1024 (the device status flag isn't raised then)
	//Happens if we are too slow with getting the data and can't use it up enough fast
	if (mpuDeviceStatus & 0x10 || mpuFIFOCount == 1024) {
		//Reset (empty) the FIFO buffer so we can keep collecting new data
		//If we don't reset the buffer it gets overflown and bits start shifting -> Random results
		mpu.resetFIFO();
	}
	//Else check if the interrupt status is decimal 1, so the DMP has it's data ready for us
	else if (mpuInterruptStatus & 0x01) {
		//Let's wait till we have the right amount of data for a valid packet ready
		while (mpuFIFOCount < mpuPacketSize)
		{
			mpuFIFOCount = mpu.getFIFOCount();
		}

		//Let's get a whole packet from the FIFO buffer and save it into the mpuFIFOData variable
		mpu.getFIFOBytes(mpuFIFOData, mpuPacketSize);

		//Let's remove the amount of read data from the FIFO count so if there's more data already available we can immediatly process it
		mpuFIFOCount -= mpuPacketSize;

		//All done, now let's calculate the correct values
		//Let the DMP calculate the quaternions from the data stored in mpuFIFOData
		mpu.dmpGetQuaternion(&mpuQuaternion, mpuFIFOData);
		//Let the DMP calculate the gravity values from quaternion
		mpu.dmpGetGravity(&mpuGravity, &mpuQuaternion);
		//Let the DMP calculate the yaw, pitch, roll values from the quaternion and gravity values
		mpu.dmpGetYawPitchRoll(yawPitchRoll, &mpuQuaternion, &mpuGravity);

		angle_X = yawPitchRoll[1] * 180 / M_PI;
		angle_Y = yawPitchRoll[2] * 180 / M_PI;
		angle_Z = yawPitchRoll[0] * 180 / M_PI;
	}

	//The microseconds since alive when the work in loop is done
	endLoopTime = micros();

	loopCycleCounter++;
	if (loopCycleCounter >= 10) {
		loopCycleCounter = 0;


		//Debug output, uncomment for some additional information

		/*Serial.print(F("pidPitch_In "));
		Serial.print(pidPitch_In);
		Serial.print(F(" -- pidRoll_In "));
		Serial.print(pidRoll_In);
		Serial.print(F(" -- pidYaw_In "));
		Serial.println(pidYaw_In);
		Serial.print(F("Motor 0 - "));
		Serial.println(motor0Power);
		Serial.print(F(" -- Motor 1 - "));
		Serial.print(motor1Power);
		Serial.print(F(" -- Motor 2 - "));
		Serial.print(motor2Power);
		Serial.print(F(" -- Motor 3 - "));
		Serial.println(motor3Power);*/
	}
}



void pinIsRising() {
	//Save the pin that has started the interrupt.
	lastInterruptedPin = PCintPort::arduinoPin;
	//Attach the falling interrupt to the triggered pin
	PCintPort::attachInterrupt(lastInterruptedPin, &pinIsFalling, FALLING);
	//Save the micros() value in the pwmStartMicroseconds ---------------------- NOT APPLICABLE ANYMORE: offset by 2 (as we are starting at PIN 2 and not 0)
	pwmStartMicroseconds[lastInterruptedPin] = micros();
}

void pinIsFalling() {
	//Save the pin that has started the interrupt
	lastInterruptedPin = PCintPort::arduinoPin;
	//Attach the rising interrupt again --> Ready for next pwm cycle
	PCintPort::attachInterrupt(lastInterruptedPin, &pinIsRising, RISING);
	//Calculate the microseconds between rising and falling interrupt of the pin. --> Raw PWM value
	receiverRawValue[lastInterruptedPin] = micros() - pwmStartMicroseconds[lastInterruptedPin];
}

void mpuInterruption() {
	mpuInterrupt = true; //New data is ready for processing in the loop
}
