/*
 *	file :			I2cDualSpeedSensor.ino
 *	Project :		Arduino Nano
 *	Created on :	14 Nov 2020
 *	Last edit :		22 Nov 2020
 *	Author :		jacob
 *
 *	Version	Date	Comment
 *	-------	----	-------
 *	0.0.1	201122	Added option to send both registers
 *					placed timings in constants
 *	0.0.1	201114	Initial version
 *
 *  Uses libraries:
 *    TimerOne
 *    Wire
 *
 */

#include <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>


const int sensorA_ = 2;    // DPIN speed sensor A
const int sensorB_ = 3;    // DPIN speed sensor B
volatile unsigned int pulsesA_ {0};
volatile unsigned int pulsesB_ {0};
volatile int RPM_A_ {0};			// Register content speed sensor A
volatile int RPM_B_ {0};			// Register content speed sensor B

const byte i2cAddress {0X11};		// I2c address for this slave
const byte configReg_ {0x00};		// Register Address of configuration (RESERVED NOT IN USE)
const byte RPM_A_Reg_ {0x01};		// Register Address of speed sensor A
const byte RPM_B_Reg_ {0x02};		// Register Address of speed sensor B
// These to calculate the RPM's the multiplication of them must be 3,000,0000
const long timerPeriode_ {500000};	// timer interval is 0.5 second 2Hz
const int pulseMult_ {6};			// PulsesPer0.5Sec / 20 * 120 = PulsesPerSec * 6 = RPM

const int maxValBufSize_ {4};		// Max value buffer size is 4. For 2 16 bits registers.
const int MSB {0};					// Most significant byte
const int LSB {1};					// Least significant byte
volatile byte registerNo_ {0};		// To store the register requested




void doCountA()		// counts from speed sensor A
{
	pulsesA_++;		// increase +1 the counter value
}

void doCountB()		// counts from speed sensor B
{
	pulsesB_++;		// increase +1 the counter value
}

void timer1Isr()
{
	Timer1.detachInterrupt();				// stop the timer
	RPM_A_ = (pulsesA_ * pulseMult_);		// Calculate RPMs
	RPM_B_ = (pulsesB_ * pulseMult_);		// Calculate RPMs
	pulsesA_ = 0;							// reset counterA to zero
	pulsesB_ = 0;							// reset counterB to zero
	Timer1.attachInterrupt(timer1Isr);		// enable the timer
}


void handleI2cReceive(int bytes) {
	// Read the first byte to determine which register is concerned
	registerNo_ = Wire.read();

	// If there is more than 1 byte, then the master is writing to the slave
	// However there is no write-able registers implemented ... yet
}


void handleI2cRequest() {
	uint8_t valueBuffer[maxValBufSize_];	// 2 bytes to send back
	int value1, value2;						// temp storage for register values
	int usedBufferSize {2};					// default 1 register is 2 bytes

	// Which register was requested
	switch (registerNo_) {
		case 1:
			value1 = RPM_A_;
			break;
		case 2:
			value1 = RPM_B_;
			break;
		case 3:
			value1 = RPM_A_;
			value2 = RPM_B_;
			usedBufferSize = 4;				// 2 registers is 4 bytes
			break;
		default:
			value1 = 0xFFFF;				// unknown register send hex FFFF
	}
	// MSB value first
	valueBuffer[MSB] = value1 >> 8;			// msb in 1st data byte
	// LSB second
	valueBuffer[LSB] = value1 & 0xFF;		// lsb in 2nd data byte

	if (registerNo_ == 3) {
		// MSB value first
		valueBuffer[MSB+2] = value2 >> 8;	// 2nd register msb in 3th data byte
		// LSB second
		valueBuffer[LSB+2] = value2 & 0xFF;	// 2nd register lsb in 4th data byte
	}

	// Send the value back
	Wire.write(valueBuffer, usedBufferSize);
}




void setup()
{
	// Setup Sensors
	pinMode(sensorA_, INPUT_PULLUP );
	pinMode(sensorB_, INPUT_PULLUP );

	// Setup timers and interrupt routines

	//	Timer1.initialize(1000000 );		// set timer period
	Timer1.initialize(timerPeriode_ );		// set timer period
	attachInterrupt(digitalPinToInterrupt(sensorA_ ), doCountA, RISING );	// increase counter when speed sensor pin goes High
	attachInterrupt(digitalPinToInterrupt(sensorB_ ), doCountB, RISING );	// increase counter when speed sensor pin goes High
	Timer1.attachInterrupt(timer1Isr );		// enable the timer1

	// Setup I2C
	Wire.begin(i2cAddress );
	Wire.onReceive(handleI2cReceive );
	Wire.onRequest(handleI2cRequest );
}

void loop()
{
	// nothing here all in interrupt service routines
}
