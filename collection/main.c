/*
 * Title:   main.c
 * Project: Flight Data Collection System
 * Author:  Martin A. DeWitt, Assistant Professor of Physics, High Point University
 * Date:    April 15, 2018
 *
 * Purpose: Allow a Raspberry Pi to collect and log data from an Adafruit BNO055 orientation sensor,
 *          two AllSensors DLHR-L10D differential pressure sensors, and a radio-controlled aircraft
 *          receiver. The Raspberry Pi, sensors, and receiver are onboard a radio-controlled
 *          ApprenticeS trainer aircraft made by Horizon Hobby.
 *
 * Method: The Adafruit BNO055 has an onboard Cortex-M0 to collect data from a 3-axis gyro, a
 *         3-axis accelerometer, and 3-axis magnetometer, perform sensor fusion, and output the
 *         resulting orientation data at 100 Hz. Therefore, the BNO055 is polled inside of a data
 *         collection loop in MAIN to retrieve the Euler angles (heading, roll, pitch). Because the
 *         BNO055 employs clock stretching, which the Raspberry Pi is not equipped to handle, it is
 *         connected via UART rather than I2C.
 *
 *         Each differential pressure sensor measures the difference between pitot and static pressures
 *         in a pitot-static tube mounted under the aircraft's wing. These pressure differences are
 *         used to determine the aircraft's airspeed and angle of attack. One pressure sensor is
 *         connected via SPI, and the second connected via I2C. The sensors are equipped with interrupt
 *         pins, which are connected to GPIO pins on the Raspberry Pi. These GPIO pins are used to
 *         detect the rising edges of interrupt signals sent by the pressure sensors once the result
 *         of a measurement is available. Interrupt service routines are used to retrieve data from the
 *         differential sensors' registers and request the start of a new pressure measurement.
 *
 *         The elevator and throttle channels of the aircraft's radio receiver are connected to
 *         Raspberry Pi GPIO pins. Both the rising and falling edges of the PWM signals sent from the
 *         aircraft's radio receiver are treated as interrupt signals. Interrupt service routines are
 *         used to record the times of these edges, which are later used to reconstruct the PWM signal
 *         and determine the commanded elevator deflection and throttle setting.
 *
 *         *NOTE: A few NON-CONSTANT GLOBAL VARIABLES are used to hold data collected by the interrupt
 *         service routines because these routines cannot have formal parameters and must be
 *         of return type VOID. The interrupt service routines are the only functions that
 *         are allowed to modify the global variables. If this code is modified, do not write
 *         any functions that modify these variables.
 *
 *         Use is made of the WiringPi library (wiringpi.com), which is a "PIN based GPIO library
 *         written in C BCM2835, BCM2836 and BCM2837 SoC devices used in all Raspberry Pi versions."
 *         Header files from this include wiringPi.h, wiringPiSPI.h, wiringPiI2C.h, and wiringPiSerial.h.
 *         The library was developed directly on Raspberry Pi and no other platforms or cross-compilers
 *         are supported by the original author.
 *
 */
 
#include<stdio.h>
#include<unistd.h>
#include<stdint.h>
#include<termios.h>
#include<math.h>
#include<wiringPi.h>
#include<wiringPiSPI.h>
#include<wiringPiI2C.h>
#include<wiringSerial.h>
#include<time.h>

#include "FlightSensors.h"  // Functions and Interrupt Service Routines for communicating with sensors

/* NON-CONST GLOBAL Variables Needed for Interrupt Service Routines:
 * These variables are only modified by the corresponding interrupt service routine. DO NOT ADD ANY FUNCTIONS
 * THAT MODIFY THESE VARIABLES.
 */
//Elevator PWM Global Variables for ISR -- ONLY MODIFIED BY elevator_isr()
static volatile int elevatorCounter;               // Counter for the number of data records collected
static volatile double elevatorTime[MAX_RECORDS];  // Array for storing PWM edge times. Index is the record number (elevatorCounter).

// Throttle PWM Global Variables for ISR -- ONLY MODIFIED BY throttle_isr()
static volatile int throttleCounter = 0;            // Counter for the number of data records collected
static volatile double throttleTime[MAX_RECORDS];   // Array for storing PWM edge times. Index is the record number (throttleCounter).

// SPI Differential Pressure Sensor Global Variables for ISR -- ONLY MODIFIED by spi_pressure_isr()
static volatile int spiCounter;  // Counter for the number of data records collected
/* spiData[i][j]: Array for storing SPI data.
 * First index is the record number (spiCounter). Second index is one of the following:
 * PRESSURE_DATA_TIME, PRESSURE_DATA_MSB, PRESSURE_DATA_CSB, PRESSURE_DATA_LSB,
 * PRESSURE_READ_ERROR, PRESSURE_REQUEST_ERROR.
 */	 
static volatile uint8_t spiData[MAX_RECORDS][MAX_VARS];
	
// I2C Differential Pressure Sensor Global Variables for ISR  -- ONLY MODIFIED by i2c_pressure_isr()
static volatile int i2cCounter;  // counter for the number of data records collected
/* i2cdata[i][j]: Array for storing I2C data.
 * First index is the record number (i2cCounter). Second index is one of the following:
 * PRESSURE_DATA_TIME, PRESSURE_DATA_MSB, PRESSURE_DATA_CSB, PRESSURE_DATA_LSB,
 * PRESSURE_READ_ERROR, PRESSURE_REQUEST_ERROR.
 */	 
static volatile uint8_t i2cData[MAX_RECORDS][MAX_VARS];

/* SPECIAL CASE: The following NON-CONST GLOBAL variable, i2cPressureId, by default as a static
 * variable, is inititalized to 0. Its value is modified ONLY ONCE in MAIN when the pressure sensor
 * is initialized on the I2C bus. Its value is then the file descriptor returned by the
 * initilazation function. It is accessed by the i2c_pressure_isr() in order to communicate with
 * the I2C pressure sensor. This value is only modified ONCE.
 */
static int i2cPressureId;


void main()
{
	/* ---------------------------
	 * BEGIN SENSOR INITIALIZATION
	 * ---------------------------
	 */
	// WiringPi Setup command so that WiringPi pin numbers can be used.
	wiringPiSetup();

	// Initialize the differential sensor on the SPI bus.
	const int SPI_PRESSURE_ID = wiringPiSPISetup(SPI_CHANNEL, SPI_CLK_SPEED);
	// Check to see whether an error or a file descriptor was returned.
	if (SPI_PRESSURE_ID == -1)
	{
	   printf("Error initializing differential pressure sensor on SPI bus!\n\n");
	}
	else
	{
	   printf("SPI: Differential pressure sensor ID = %d\n\n", SPI_PRESSURE_ID);
	}

	// Initialize the differential pressure sensor on the I2C bus.
	i2cPressureId = wiringPiI2CSetup(PRESSURE_SENSOR_I2C_ADDR);
	// Check to see whether an error or a file descriptor was returned.
	if (i2cPressureId == -1)
	{
	   printf("Error initializing differential pressure sensor on I2C bus!\n\n");
	}
	else
	{
	   printf("I2C: Differential pressure sensor ID = %d\n\n", i2cPressureId);
	}

	// Initialize the BNO055 orientation sensor on UART.
	const int UART_IMU_ID = serialOpen("/dev/ttyAMA0", IMU_BAUDRATE);
	// Check to see whether an error or a file descriptor was returned.
	if(UART_IMU_ID == -1)
	{
	   printf("Error initializing the UART interfact with IMU--BNO055 Sensor!\n");
	}
	else
	{
		printf("UART: BNO055 orientation sensor Id = %d\n\n", UART_IMU_ID)
	}

	// Use WiringPi library command to flush UART IO buffers.
	serialFlush(UART_IMU_ID);	// flush pending data
	delay(1000);				// pause 1 second before attempting to read/write the BNO055

	// Declare and initialize variables for holding UART error codes.
	int8_t uartErrorLog[UART_ERROR_LOG_SIZE];   	// Array to hold UART error codes
	for(size_t i = 0; i < UART_ERROR_LOG_SIZE; i++)	// Initialize all UART error log entries to 0
	{
		uartErrorLog[i] = 0;
	}
	size_t uartErrorLogCounter = 0;		// Initialize the UART error log counter.
	
	// Set the operating mode of the BNO055 orientation sensor to NDOF_FUSION_MODE.
	start_imu_fusion_mode(UART_IMU_ID, uartErrorLog, &uartErrorLogCounter)

	// Calibrate the BNO055's magnetometer, accelerometer, and gyro.
	calibrate_imu_sensors(const int UART_IMU_ID, int8_t *uartErrorLog, size_t *uartErrorLogCounter);

	// Declare the pins on the Raspberry Pi used for interrupt signals as INPUT. 
	pinMode(SPI_PRESSURE_INTERRUPT_PIN, INPUT);
	pinMode(I2C_PRESSURE_INTERRUPT_PIN, INPUT);
	pinMode(PWM_ELEVATOR_INTERRUPT_PIN, INPUT);
	pinMode(PWM_THROTTLE_INTERRUPT_PIN, INPUT);

	// Assign interrupt service routine functions to each interrupt.
	wiringPiISR(SPI_PRESSURE_INTERRUPT_PIN, INT_EDGE_RISING, &spi_pressure_isr);
	wiringPiISR(I2C_PRESSURE_INTERRUPT_PIN, INT_EDGE_RISING,&i2c_pressure_isr);
	wiringPiISR(PWM_ELEVATOR_INTERRUPT_PIN, INT_EDGE_BOTH,&elevator_isr);
	wiringPiISR(PWM_THROTTLE_INTERRUPT_PIN, INT_EDGE_BOTH,&throttle_isr);

	// Request inital pressure measurements on the SPI bus and I2C bus and check for initialization errors.
	if(request_new_spi_measurement() == -1)
	{
		printf("Error requesting initial measurement from pressure sensor on SPI bus!");
	}

	if(request_new_i2c_measurement() == -1)
	{
		printf("Error requesting intital measurement from pressure sensor on SPI bus!");
	}
	/* -------------------------
	 * END SENSOR INITIALIZATION
	 * -------------------------
	 */


	/* -----------------------------
	 * START DATA COLLECTION POLLING
	 * -----------------------------
	 */
	// Declare arrays for storing BNO055 IMU data
	uint8_t imuBuffer[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	double timeImu[MAX_SIZE];
	uint8_t EulerHeadingLSB[MAX_SIZE];
	uint8_t EulerHeadingMSB[MAX_SIZE];
	uint8_t EulerRollLSB[MAX_SIZE];
	uint8_t EulerRollMSB[MAX_SIZE];
	uint8_t EulerPitchLSB[MAX_SIZE];
	uint8_t EulerPitchMSB[MAX_SIZE];

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME,&ts);
	
	printf("Data Collection Start Time (s) = %lf\n",(double)ts.tv_sec + (double)ts.tv_nsec/1000000000);

	for(uint32_t i = 0; i < (uint32_t)MAX_SIZE; i++)
	{
		clock_gettime(CLOCK_REALTIME,&ts);
		time_Imu[i] = (double)ts.tv_sec + (double)ts.tv_nsec*1.0e-9;
		
		if(get_imu_data(UART_IMU_ID, imuBuffer, uartErrorLog, uartErrorLogCounter) == 0)
		{
			EulerHeadingLSB[i] = imuBuffer[0];
			EulerHeadingMSB[i] = imuBuffer[1];
			EulerRollLSB[i]    = imuBuffer[2];
			EulerRollMSB[i]    = imuBuffer[3];
			EulerPitchLSB[i]   = imuBuffer[4];
			EulerPitchMSB[i]   = imuBuffer[5];
		}
		else
		{
			EulerHeadingLSB[i] = 255;
			EulerHeadingMSB[i] = 255;
			EulerRollLSB[i]    = 255;
			EulerRollMSB[i]    = 255;
			EulerPitchLSB[i]   = 255;
			EulerPitchMSB[i]   = 255;
		}
		
	}

	clock_gettime(CLOCK_REALTIME,&ts);
	printf("Data Collection End Time (s) = %lf \n",(double)ts.tv_sec + (double)ts.tv_nsec/1000000000);
	printf("Final SPI Counter Value: %d\n\n", spiCounter);
	printf("Final I2C Counter Value: %d\n\n", i2cCounter);
	printf("Final UART Error Log Counter %u\n\n", *uartErrorLogCounter);

	/* ---------------------------
	 * END DATA COLLECTION POLLING
	 * ---------------------------
	 */


	/* -----------------------------------
	 * BEGIN FINAL CALCULTIONS AND LOGGING
	 * -----------------------------------
	 */
	 
	/* Calculate actual pressure values in Pascals(Pa) from sensor readings
	 * and heading, roll, and pitch from IMU readings.
	 */

	// Variable declarations for final pressure calculations
	double dp1[MAX_SIZE];   // Array for storing final pressure values from SPI sensors
	double dp2[MAX_SIZE];   // Array for storing final pressure values from I2C sensors
	uint32_t pOutDig;       // P_OUT_DIG is the unsinged 24-bit raw measurement (see pressure sensor datasheet)
	
	// Variavle declatarions for final Euler angle calculations
	int16_t heading_raw;        // Signed 16-bit raw measurement value from BNO055
	int16_t roll_raw;           // Signed 16-bit raw measurement value from BNO055
	int16_t pitch_raw;          // Signed 16-bit raw measurement value from BNO055
	double heading[MAX_SIZE];   // Array for storing final heading in degrees
	double roll[MAX_SIZE];      // Array for storing final roll in degrees
	double pitch[MAX_SIZE];     // Array for storing final pitch in degrees
	
	/* Use raw measurements to caluculate final data values. Equations are taken from the DLHR-L10D
	 * pressure sensor and the BNO055 orientation sensor datasheets.
	 */
	for(uint32_t j = 0; j < (uint32_t)MAX_SIZE; j++)
	{
		pOutDig = (spiData[j][PRESSURE_DATA_MSB] << 16) | (spiData[j][PRESSURE_DATA_CSB] << 8) | (spiData[j][PRESSURE_DATA_LSB]);
		dp1[j] = 248.84*1.25*((pOutDig - 0.5*0x1000000)/0x1000000)*20.0;

		pOutDig = (i2cData[j][PRESSURE_DATA_MSB] << 16) | (i2cData[j][PRESSURE_DATA_CSB] << 8) | (i2cData[j][PRESSURE_DATA_LSB]);
		dp2[j] = 248.84*1.25*((pOutDig - 0.5*0x1000000)/0x1000000)*20.0;

		heading_raw = (EulerHeadingMSB[j] << 8) | EulerHeadingLSB[j];
		roll_raw = (EUL_ROLL_MSB[j]<<8) | EUL_ROLL_LSB[j];
		pitch_raw = (EUL_PITCH_MSB[j]<<8) | EUL_PITCH_LSB[j];
		heading[j] = heading_raw/16.0;
		roll[j] = roll_raw/16.0;
		pitch[j] = pitch_raw/16.0;
	}

	// Write final data to CSV files.
	FILE *fp;
	uint32_t k = 0;
	
	fp = fopen("dp1data.csv", "w");
	fprintf(fp,"time(s),dP1(Pa)\n");
	for(k = 0; k < (uint32_t)spiCounter; k++)
	{
	   fprintf(fp, "%lf,%lf\n", spiData[k][PRESSURE_DATA_TIME], dp1[k]);
	}
	fclose(fp);


	fp = fopen("dp2data.csv","w");
	fprintf(fp, "time(s),dP2(Pa)\n");
	for(k = 0; k < (uint32_t)i2cCounter; k++)
	{
	   fprintf(fp, "%lf,%lf\n", i2cData[k][PRESSURE_DATA_TIME], dp2[k]);
	}
	fclose(fp);


	fp = fopen("imudata.csv","w");
	fprintf(fp, "time(s),heading(deg),roll(deg),pitch(deg)\n");
	for(k = 0 ; k < (uint32_t)MAX_SIZE; k++)
	{
	   fprintf(fp, "%lf,%lf,%lf,%lf\n", time_IMU[k], heading[k], roll[k], pitch[k]);
	}
	fclose(fp);


	fp = fopen("elevatordata.csv","w");
	fprintf(fp, "time(s)\n");
	for(k = 0; k < (uint32_t)elevatorCounter; k++)
	{
	   fprintf(fp, "%lf\n", elevatorTime[k]);
	}
	fclose(fp);


	fp = fopen("throttledata.csv","w");
	fprintf(fp, "time(s)\n");
	for(k = 0; k < (uint32_t)throttleCounter; k++)
	{
	   fprintf(fp,"%lf\n",throttleTime[k]);
	}
	fclose(fp);
	
	fp = fopen("spierrors.csv","w");
	fprintf(fp, "time(s),read_error, request_error\n");
	for(k = 0; k < (uint32_t)spiCounter; k++)
	{
		fprintf(fp,"%lf,%d,%d\n", spiData[k][PRESSURE_DATA_TIME], spiData[k][PRESSURE_READ_ERROR], spi[k][PRESSURE_REQUEST_ERROR]);
	}
	fclose(fp);
	
	fp = fopen("i2cerrors.csv","w");
	fprintf(fp, "time(s),read_error, request_error\n");
	for(k = 0; k < (uint32_t)i2cCounter; k++)
	{
		fprintf(fp,"%lf,%d,%d\n", i2cData[k][PRESSURE_DATA_TIME], i2cData[k][PRESSURE_READ_ERROR], i2cData[k][PRESSURE_REQUEST_ERROR]);
	}
	fclose(fp);
	
	fp = fopen("uarterrors.csv","w");
	fprintf(fp, "error_msg\n");
	for(k = 0; k < (uint32_t)uartErrorLogCounter; k++)
	{
		switch(uartErrorLog[k]){
			case BNO055_READ_FAIL:
				fprintf(fp, "READ_FAIL\n");
				break;
			case BNO055_WRITE_FAIL:
				fprintf(fp, "WRITE_FAIL\n");
				break;
			case BNO055_REGMAP_INVALID_ADDRESS:
				fprintf(fp, "REGMAP_INVALID_ADDRESS\n");
				break;
			case BNO055_REGMAP_WRITE_DISABLED:
				fprintf(fp, "REGMAP_WRITE_DISABLED\n");
				break;
			case BNO055_WRONG_START_BYTE:
				fprintf(fp, "WRONG_START_BYTE\n");
				break;
			case BNO055_BUS_OVER_RUN_ERROR:
				fprintf(fp, "BUS_OVERRUN_ERROR\n");
				break;
			case BNO055_MAX_LENGTH_ERROR:
				fprintf(fp, "MAX_LENGTH_ERROR\n");
				break;
			case BNO055_MIN_LENGTH_ERROR:
				fprintf(fp, "MIN_LENGTH_ERROR\n");
				break;
			case BNO055_RECEIVE_CHARACTER_TIMEOUT:
				fprintf(fp, "RECEIVE_CHARACTER_TIMEOUT\n");
				break;
			case UART_UNDEFINED_RESPONSE:
				fprintf(fp, "UART_UNDEFINED_RESPONSE\n");
				break;
			case UART_WRITE_ERROR:
				fprintf(fp, "UART_WRITE_ERROR\n");
				break;
			case UART_INCOMPLETE_WRITE:
				fprintf(fp, "UART_INCOMPLETE_WRITE\n");
				break;
			case UART_READ_ERROR:
				fprintf(fp, "UART_READ_ERROR\n");
				break;
			case UART_INCOMPLETE_READ:
				fprintf(fp, "UART_INCOMPLETE_READ\n");
				break;
			case UART_SEND_ERROR:
				fprintf(fp, "UART_SEND_ERROR\n");
				break;
			default:
				fprintf(fp, "unknown value\n");
				break;
		}
	}
	fclose(fp);
	
	/* ---------------------------------
	 * END FINAL CALCULTIONS AND LOGGING
	 * ---------------------------------
	 */
}
