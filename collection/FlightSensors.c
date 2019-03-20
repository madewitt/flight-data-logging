/*
 * Title:   FlightSensors.c
 * Project: Flight Data Collection System
 * Author:  Martin A. DeWitt, Assistant Professor of Physics, High Point University
 * Date:    April 15, 2018
 *
 * Purpose: Functions and interrupt service routines for communicating with sensors.
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

#include "FlightSensors.h"

/* MAX_SIZE: number of times that the BNO055 orientation sensor is polled for data
 * A rough approximation is MAX_SIZE = (100 Hz data rate)(60 s/min)x(flight time in minutes)
 * MAX_SIZE = 120000 translates to roughly 20 minutes of prep/flight time once the sensors
 * are calibrated.
 */
#define MAX_SIZE = 120000

// Define Interrupt Pin Numbers for PWM signal and differential pressure sensors
#define SPI_PRESSURE_INTERRUPT_PIN 0   // Pressure SPI interrupt pin is (#11)--GPIO17 --> WiringPi Pin 0
#define I2C_PRESSURE_INTERRUPT_PIN 1   // Pressure I2C interrupt pin is (#12)--GPIO18 --> WiringPi Pin 1
#define PWM_ELEVATOR_INTERRUPT_PIN 2   // Elevator GPIO interrupt pin is (#13)--GPIO27 --> WiringPi Pin 2
#define PWM_THROTTLE_INTERRUPT_PIN 3   // Throttle GPIO interrupt pin is (#15)--GPIO22 --> WiringPi Pin 3

// Define constatns related to differential pressure sensors
#define MAX_RECORDS 120000      // Maximum number of records that can be stored in a pressure data array
#define MAX_VARS 6              // Maximum number of varaibles stored in a pressure data array
#define PRESSURE_DATA_TIME 0    // Second index of pressure data 2D arrays where time is stored
#define PRESSURE_DATA_MSB 1     // Second index of pressure data 2D arrays where the MSB of the measurement is stored
#define PRESSURE_DATA_CSB 2     // Second index of pressure data 2D arrays where the CSB of the measurement is stored
#define PRESSURE_DATA_LSB 3     // Second index of pressure data 2D arrays where the LSB of the measurement is stored
#define PRESSURE_READ_ERROR 4   // Second index of pressure data 2D arrays where a pressure read error is indicated
#define PRESURE_REQUEST_ERROR 5 // Second index of pressure data 2D arrays where an error requesting to start a new measurement is stored
#define SPI_CHANNEL 0           // SPI channel on the Raspberry Pi to which the pressure sensor is connected (0 or 1)
#define SPI_CLK_SPEED 500000    // SPI clock speed (frequency in Hz)

// Maximum number of records that can be stored in the UART error log
#define UART_ERROR_LOG_SIZE 20000

// Official BNO055 error codes:
// Code written to BYTE-0 of uartResponse upon successful request to read data from the BNO055. BYTE-1 of uartResponse is undefined.
#define BNO055_READ_SUCCESS 0xBB
// Code written to BYTE-0 of uartResponse. Indicates that BYTE-1 should be read for the specific code.
#define BNO055_ACKNOWLEDGE_RESPONSE 0xEE
// Codes written to BYTE-1 of uartResponse when BYTE-0 of uartResponse is 0xEE.
#define BNO055_WRITE_SUCCESS 0x01
#define BNO055_READ_FAIL 0x02
#define BNO055_WRITE_FAIL 0x03
#define BNO055_REGMAP_INVALID_ADDRESS 0x04
#define BNO055_REGMAP_WRITE_DISABLED 0x05
#define BNO055_WRONG_START_BYTE 0x06
#define BNO055_BUS_OVER_RUN_ERROR 0x07
#define BNO055_MAX_LENGTH_ERROR 0x08
#define BNO055_MIN_LENGTH_ERROR 0x09
#define BNO055_RECEIVE_CHARACTER_TIMEOUT 0x0A

// Custom error codes not part of the BNO055 itself:
// Written to error log if BYTE-0 of uartResponse is neither 0xBB or 0xEE
#define UART_UNDEFINED_RESPONSE 0x0B
// Used when a command sent to the BNO055 is broken into single bytes. Indicates an error occurred when attemting a single byte write.
#define UART_WRITE_ERROR 0x0C
// Used when a command sent to the BNO055 is broken into single bytes. Indicates the number of bytes written was not 1.
#define UART_INCOMPLETE_WRITE 0x0D
// Used when a read() command fails on UART after a successful request to read data from the BNO055. 
#define UART_READ_ERROR 0x0E
// Used when a read() command fails to retreive the expected number of data bytes after a successful request to read data from the BNO055.   
#define UART_INCOMPLETE_READ 0x0F
// Indicates that the uart_send() function experienced a failure.
#define UART_SEND_ERROR 0x10


#define PRESSURE_SENSOR_I2C_ADDR 0x29   // The I2C address of the differential pressure sensor
#define IMU_BAUDRATE 115200             // Baudrate to use for communicating with the BNO055 on UART.


/* 
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: elevator_isr
 * --------------------------------------------------------------------------------------------------------
 *
 * Interrupt service routine that records the time of a rising or falling edge of a the PWM signal
 * sent to the aircraft's elevator servo.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS: None
 *
 * ---------------------------------------------------------------------------------------------------------
 * RETURNS: N/A
 *
 * NOTE: elevatorTime (array) and elevatorCounter are non-const global variables. This interrupt service
 *       routine is the only function that modifies the values of these variables. In MAIN, the values
 *       stored in these variables are read and stored in CSV files.
 *
 * ---------------------------------------------------------------------------------------------------------
 */
void elevator_isr(void)
{
	static volatile struct timespec elevatorTimespec;	// Time: whole seconds stored in tv_sec and fractional part in tv_nsec as nanoseconds
	// Record time of rising or falling edge of PWM signal
	clock_gettime(CLOCK_REALTIME, &elevatorTimespec);
	elevatorTime[elevatorCounter] = (double)elevatorTimespec.tv_sec + (double)elevatorTimespec.tv_nsec/1000000000;
	elevatorCounter++;
}


/* 
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: throttle_isr
 * --------------------------------------------------------------------------------------------------------
 *
 * Interrupt service routine that records the time of a rising or falling edge of a the PWM signal
 * sent to the aircraft's electronic speed controller (ESC), which controls the electric motor.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS: None
 *
 * ---------------------------------------------------------------------------------------------------------
 *
 * RETURNS: N/A
 *
 * NOTE:  throttleTime (array) and throttleCounter are non-const global variables. This interrupt service
 *        outine is the only function that modifies the values of these variables. In MAIN, the values
 *        stored in these variables are read and stored in CSV files.
 *
 * ---------------------------------------------------------------------------------------------------------
 */
void throttle_isr(void)
{
	static volatile struct timespec throttleTimespec;	// Time: whole seconds stored in tv_sec and fractional part in tv_nsec as nanoseconds
	// Record time of rising or falling edge of PWM signal
	clock_gettime(CLOCK_REALTIME, &throttleTimespec);
	throttleTime[throttleCounter] = (double)throttleTimespec.tv_sec + (double)throttleTimespec.tv_nsec/1000000000;
	throttleCounter++;
}


/* 
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: request_new_spi_measurement
 * --------------------------------------------------------------------------------------------------------
 *
 * Sends a command to the differential pressure sensor on the SPI bus to start a new measurement.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS: None
 *
 * ---------------------------------------------------------------------------------------------------------
 *
 * RETURNS: Returns 0 if the new measurement request is successful; returns -1 if there is an error
 * requesting the measurement
 *
 * ---------------------------------------------------------------------------------------------------------
 */
int request_new_spi_measurement(void)
{
	static const size_t REQUEST_PRESSURE_CMD_LENGTH = 3;
	static const uint8_t REQUEST_PRESSURE_CMD[REQUEST_PRESSURE_CMD_LENGTH] = {0xAE, 0x00, 0x00};
	return ( (wiringPiSPIDataRW(SPI_CHANNEL, REQUEST_PRESSURE_CMD, REQUEST_PRESSURE_CMD_LENGTH) < 0) ? -1 : 0 );
}


/*
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: get_spi_pressure_data
 * --------------------------------------------------------------------------------------------------------
 *
 * Sends a read request to the pressure sensor on the SPI bus and reads the pressure data into a buffer.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS:
 *     buffer: Pointer to a buffer defined in the calling function. The read command is stored in this
 *             buffer and is sent to the sensor. The buffer is overwritten with the returned data.
 *
 *     bufferLength: Size of the buffer. 	
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * RETURNS: Returns 0 if the data retrieval is successful; -1 if an error occurs.
 *
 * --------------------------------------------------------------------------------------------------------
 */
int get_spi_pressure_data(uint8_t *buffer, size_t bufferLength)
{
	static const size_t READ_PRESSURE_CMD_LENGTH = 7;
	static const uint8_t READ_PRESSURE_CMD[READ_PRESSURE_CMD_LENGTH] = {0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	// Describe structure of read command for SPI
	for(size_t i = 0; i < READ_PRESSURE_CMD_LENGTH; i++)
		buffer[i] = READ_PRESSURE_CMD[i];
	
	// Request data from the pressure sensor on the SPI bus
	return ((wiringPiSPIDataRW(SPI_CHANNEL, buffer, bufferLength) < 0) ? -1 : 0);
}


/*
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: spi_pressure_isr
 * --------------------------------------------------------------------------------------------------------
 *
 * Interrupt service routine that collects and records data for the pressure sensor on the SPI bus.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS: None			
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * RETURNS: N/A
 *
 * NOTE:   spiData(array) and spiCounter are non-const global variables used to store the pressure data.
 *         This is the ONLY function that modifies these variables. In MAIN, the values stored in these
 *         variables are read and stored in CSV files.
 *
 * --------------------------------------------------------------------------------------------------------
 */
void spi_pressure_isr(void)
{
	static volatile struct timespec spiTimespec;
	// Record the time of the measurement
	clock_gettime(CLOCK_REALTIME,&spiTimespec);
	spiData[spiCounter][PRESSURE_DATA_TIME] = (double)spiTimespec.tv_sec + (double)spiTimespec.tv_nsec/1000000000;

	/* Read pressure sensor data. Results are stored in a 7 byte "buffer" after call to get_spi_pressure_data.
	 * buffer[0] -> status byte of the sensor
	 * buffer[1] -> MSB of 24-bit pressure measurement
	 * buffer[2] -> CSB of 24-bit pressure measurement
	 * buffer[3] -> LSB of 24-bit pressure measurement
	 * buffer[4] -> MSB of 24-bit temperature measurement
	 * buffer[5] -> CSB of 24-bit temperature measurement
	 * buffer[6] -> LSB of 24-bit temperature measurement
	*/
	static const size_t bufferLength = 7;   // There are seven bytes of data returned by the pressure sensor
	static uint8_t buffer[bufferLength];    // Buffer to hold returned data
	static int spi_return_value;            // Holds return value of a function call
	
	spi_return_value = get_spi_pressure_data(buffer, bufferLength);

	// Check to see if there is an error reading the data
	spiData[spiCounter][PRESSURE_READ_ERROR] = ((spi_return_value == -1) ? 1 : 0);
   
	// Store pressure data in arrays. Ignore temperature data since it is not used for pressure compensation.
	spiData[spiCounter][PRESSURE_DATA_MSB] = buffer[1];
	spiData[spiCounter][PRESSURE_DATA_CSB] = buffer[2];
	spiData[spiCounter][PRESSURE_DATA_LSB] = buffer[3];

	// Request the start of a new measurement
	spi_return_value = request_new_spi_measurement();
   
	// Check to see if there is an error requesting a new measurement
	spiData[spiCounter][PRESSURE_REQUEST_ERROR]= ( (spi_return_value == -1) ? 1 : 0);

	spiCounter++;
}


/* 
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: request_new_i2c_measurement
 * --------------------------------------------------------------------------------------------------------
 *
 * Sends a command to the differential pressure sensor on the I2C bus to start a new measurement.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS: None
 *
 * ---------------------------------------------------------------------------------------------------------
 *
 * RETURNS: Returns 0 if the new measurement request is successful; returns -1 if there is an error
 *          requesting the measurement
 *
 * ---------------------------------------------------------------------------------------------------------
 */
int request_new_i2c_measurement(void)
{
	static const size_t REQUEST_PRESSURE_CMD_LENGTH = 3;
	static const uint8_t REQUEST_PRESSURE_CMD[REQUEST_PRESSURE_CMD_LENGTH] = {0xAE, 0x00, 0x00};
	return write(i2cPressureId, REQUEST_PRESSURE_CMD, REQUEST_PRESS_CMD_LENGTH);
}


/*
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: i2c_pressure_isr
 * --------------------------------------------------------------------------------------------------------
 *
 * Interrupt service routine to collect and record data from the pressure sensor on the I2C bus.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS: None
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * RETURNS: N/A
 *
 * NOTE:   i2cData(array) and i2cCounter are non-const global variables used to store the pressure data.
 *         This is the ONLY function that modifies these variables. In MAIN, the values stored in these
 *         variables are read and stored in CSV files.
 *
 * --------------------------------------------------------------------------------------------------------
 */
void i2c_pressure_isr(void)
{
	static volatile struct timespec i2cTimespec;
	clock_gettime(CLOCK_REALTIME,&i2cTimespec);
	i2cData[i2cCounter][PRESSURE_DATA_TIME] = (double)i2cTimespec.tv_sec + (double)i2cTimespec.tv_nsec/1000000000;
	
	// Read pressure and temperature data
	static const size_t numberOfBytes = 7;
	static uint8_t buffer[numberOfBytes] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static volatile int i2c_return_value = read(i2cPressureId, buffer, numberOfBytes);
   
	// Check for an error reading data
	i2cData[i2cCounter][PRESSURE_READ_ERROR] = ( (i2c_return_value == -1) ? 1 : 0 );

	// Store data in arrays
	i2cData[i2cCounter][PRESSURE_DATA_MSB] = i2cBuffer[1];
	i2cData[i2cCounter][PRESSURE_DATA_CSB] = i2cBuffer[2];
	i2cData[i2cCounter][PRESSURE_DATA_LSB] = i2cBuffer[3];
   
	// Request a new measurement
	i2c_return_value = request_new_i2c_measurement();

	// Check to see if there is an error requesting a new measurement
	i2cData[i2cCounter][PRESSURE_REQUEST_ERROR]= ( (i2c_return_value == -1) ? 1 : 0);
   
	i2cCounter++;
}


/*
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: uart_send
 * --------------------------------------------------------------------------------------------------------
 *
 * A helper function that sends serial data to the BNO055 orientation sensor connected via UART. Due to
 * the tendency of the BNO055 to experience bus overruns, the command from the buffer is sent one byte at
 * a time with a 2 millisecond delay in between each byte.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS:
 *     uartId:              Id of the device connected via UART.
 *
 *     uartCmd:             Pointer to a buffer that holds the multi-byte command to be sent to the sensor.
 *
 *     uartCmdLength:       The number of bytes in uartCmd.
 *
 *     uartResponse:        Pointer to a buffer in which to store the sensor's response to the command sent.
 *
 *     uartResponseLength:  The number of bytes that can be stored in uartResponse.
 *
 *     uartErrorLog:        Pointer to an array in which to store error UART error codes.
 *
 *     uartErrorLogCounter: Pointer to a counter that keeps track of the number of UART error codes
 *                           recorded.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * RETURNS: Returns 0 if the process is successful; -1 if an error occurs. See uartErrorLog for details.
 *
 * --------------------------------------------------------------------------------------------------------
 */
int8_t uart_send(int8_t uartId, uint8_t *uartCmd, int uartCmdLength, uint8_t *uartResponse, int8_t uartResponseLength, int8_t *uartErrorLog, size_t *uartErrorLogCounter)
{
   static int8_t uart_return_value;
   static uint8_t commandByte[1];
   
   /* The BNO055 is prone to BUS_OVERRUN errors.  To prevent this, when writing to the device
    * the multi-byte write command must be broken into single byte commands, each sent with a 2ms
    * delay in between.  This is accomplished with the FOR loop directly below.
    */
   for(size_t j = 0; j < uartCmdLength; j++)
   {
       commandByte[0] = uartCmd[j];
       uart_return_value = write(uartId, commandByte, 1);
       if(uart_return_value == -1)
		   uartErrorLog[(*uartErrorLogCounter)++] = UART_WRITE_ERROR;
       else if(uart_return_value != 1)
       {
		   uartErrorLog[(*uartErrorLogCounter)++] = UART_INCOMPLETE_WRITE;
       }
       delay(2);
   }
   uart_return_value = read(uartId, uartResponse, uartResponseLength);
   if(uart_return_value == -1)
   {
      uartErrorLog[(*uartErrorLogCounter)++] = UART_READ_ERROR;
   }


   if( (uartResponse[0] == BNO055_READ_SUCCESS) || ((uartResponse[0] == BNO055_ACKNOWLEDGE_RESPONSE) && (uartResponse[1] == BNO055_WRITE_SUCCESS)) )
   {
      return 0;
   }
   else if(uartResponse[0] == BNO055_ACKNOWLEDGE_RESPONSE)
   {
		uartErrorLog[(*uartErrorLogCounter)++] = uartResponse[1];
		return -1;
   }
	else
   {
		uartErrorLog((*uartErrorLogCounter)++) = UART_UNDEFINED_RESPONSE;
		return -1;
   }
}


/*
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: start_imu_fusion_mode
 * --------------------------------------------------------------------------------------------------------
 *
 * Reads the operating mode register (OPR_MODE located at address 0x3D) of the BNO055 orientation sensor
 * and checks whether it is set to fusion mode or not. If not already in fusion mode, the register is
 * modified to put the sensor in fusion mode.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS:
 *     UART_IMU_ID:         The device ID of the BNO055 sensor on UART.
 *
 *     uartErrorLog:        Pointer to an array for storing error codes.
 *
 *     uartErrorLogCounter: Pointer to a counter to keep track of the number of error codes recorded.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * RETURNS: N/A
 *
 * --------------------------------------------------------------------------------------------------------
 */
void start_imu_fusion_mode(const int UART_IMU_ID, int8_t *uartErrorLog, size_t *uartErrorLogCounter)
{
	uint8_t OPR_MODE = 0;    // Operating mode of the BNO055. Stored in register address Page 0: 0x3D
	const int MAX_IMU_CMD_LENGTH = 5;
	uint8_t imuCmd[MAX_IMU_CMD_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00};
	int imuCmdLength = 0;
	int IMU_RESPONSE_LENGTH = 2;
	uint8_t imuResponse[IMU_RESPONSE_LENGTH] = {0x00, 0x00};
	uint8_t numberOfBytes = 0;
	
	// Send command to read the IMU's Operating Mode register 0x3D
	imuCmd[0] = 0xAA;   // Initial byte for any UART command.
	imuCmd[1] = 0x01;   // Value of 0x01 means this is a READ command.
	imuCmd[2] = 0x3D;   // The address of the register to be read.
	imuCmd[3] = 0x01;   // Number of bytes to be read.
	imuCmdLength = 4;   // Number of bytes in the command.
	
	int8_t imu_return_value;
	imu_return_value = uart_send(UART_IMU_ID, imuCmd, imuCmdLength, imuResponse, IMU_RESPONSE_LENGTH, uartErrorLog, uartErrorLogCounter);
	if(imu_return_value == 0)
	{
		numberOfBytes = imuResponse[1];
		read(UART_IMU_ID, &OPR_MODE, numberOfBytes);
		printf("OPR_MODE = %u\n", OPR_MODE);
		
		// If IMU is not in Fusion Mode, initialize fusion mode.
	    if(OPR_MODE != 0x1C)
	    {
		    // Write to OPR_MODE to initalize NDOF Fusion Mode
		    imuCmd[0] = 0xAA;            // Initial byte for any UART command.
		    imuCmd[1] = 0x00;            // Value of 0x00 means this is a WRITE command.
		    imuCmd[2] = 0x3D;            // The address of the register to be written to.
		    imuCmd[3] = 0x01;            // Number of bytes to be written.
		    imuCmd[4] = OPR_MODE | 0x0C; // Data to be written.
		    imuCmdLength = 5;            // Number of bytes in the command.
			int8_t imu_return_value2;
		    imu_return_value2 = uart_send(UART_IMU_ID, imuCmd, imuCmdLength, imuResponse, IMU_RESPONSE_LENGTH, uartErrorLog, uartErrorLogCounter);
			if(imu_return_value2 == 0)
			{
				printf("BNO055 Orientation sensor now running in fusion mode.");
			}
			else
			{
				printf("Error setting the operating mode of the BNO055 orientation sensor. See the UART Error Log.");
			}
	    }
	    else
	    {
		    printf("Already running in Fusion Mode.\n");
	    }
	    // BN0055 datasheet states it takes 19ms to change operating mode. Wait 30ms for good measure.
	    delay(30);
		
	}
	else
	{
		printf("Error reading the operating mode of the BNO055 Orientation Sensor. See UART Error Log.");
	}
}


/*
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: calibrate_imu_sensors
 * --------------------------------------------------------------------------------------------------------
 *
 * A loop that reads the calibration status register (CALIB_STAT at address 0x35) of the BNO055 orientation
 * sensor and outputs the result to STDOUT until the sensors are fully calibrated. During this time, the
 * user must do the following:
 *     1. Leave the aircraft still on a level surface for several seconds.
 *     2. Orient the aircraft such that the nose points forward and the right wing
 *		   points downward and hold still for 3-4 seconds.
 *     3. Orient the aircraft such that the nose points forward and the top of the
 *		   aircraft points downward and hold still for 3-4 seconds.
 *     3. Orient the aircraft such that the nose points forward and the left wing
 *		   points downward and hold still for 3-4 seconds.
 *     4. Orient the aircraft such that the nose points upward and hold for 3-4 seconds.
 *     5. Orient the aircraft such that the nose points downward and hold for 3-4 seconds.
 *     6. Repeat these steps until the calibration is complete. 
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS:
 *     UART_IMU_ID:         The device ID of the BNO055 sensor on UART.
 *
 *     uartErrorLog:        Pointer to an array for storing error codes.
 *
 *     uartErrorLogCounter: Pointer to a counter to keep track of the number of error codes recorded.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * RETURNS: N/A
 *
 * --------------------------------------------------------------------------------------------------------
 */
void calibrate_imu_sensors(const int UART_IMU_ID, int8_t *uartErrorLog, size_t *uartErrorLogCounter)
{
	uint8_t CALIB_STAT = 0;    // Calibration status of the BNO055. Stored at register address Page 0: 0x35
	const int imuCmdLength = 4;
	uint8_t imuCmd[imuCmdLength] = {0x00, 0x00, 0x00, 0x00};
	int8_t imu_return_value = -1;
	uint8_t numberOfBytes = 0;
	const int IMU_RESPONSE_LENGTH = 2;
	uint8_t imuResponse[IMU_RESPONSE_LENGTH] = {0x00, 0x00};
	
	/* Send command to read the Calibration Status register 0x35
	 * CALIB_STAT is a single byte: 
	 * bits 7&6 = System Calibration Status (0 to 3 where 3 is fully calibrated)
	 * bits 5&4 = Gyroscope Calibration Status (0 to 3 where 3 is fully calibrated) 
	 * bits 3&2 = Accelerometer Calibration Status (0 to 3 where 3 is fully calibrated)
	 * bits 0&1 = Magnetometer Calibration Status (0 to 3 where 3 is fully calibrated)
	 */
	imuCmd[0] = 0xAA;   // Initial byte for any BNO055 UART command is 0xAA.
	imuCmd[1] = 0x01;   // Value of 0x01 means this is a READ command.
	imuCmd[2] = 0x35;   // The address of the register to be read.
	imuCmd[3] = 0x01;   // Number of bytes to be read.

	do
	{
		imu_return_value = uart_send(UART_IMU_ID, imuCmd, imuCmdLength, imuResponse, IMU_RESPONSE_LENGTH, uartErrorLog, uartErrorLogCounter);
		if(imu_return_value == 0)
		{
			numberOfBytes = uartResponse[1];
			read(UART_IMU_ID, &CALIB_STAT, numberOfBytes);
			if ( (CALIB_STAT & 0x03) != 0x03)
			{
				printf("MAGNETOMETER not fully calibrated.");
			}
			if ( (CALIB_STAT & 0x0C) != 0x0C)
			{
				printf("ACCELEROMETER not fully calibrated.");
			}
			if ( (CALIB_STAT & 0x30) != 0x30)
			{
				printf("GYRO not fully calibrated.");
			}
			printf("CALIB_STAT = %u\n", CALIB_STAT);
				delay(3000);
		}
		else
		{
			printf("Error reading the Calibration Status of the BNO055 Orientation Sensor. See UART Error Log.");
		}
		

	}while( (CALIB_STAT & 0x3F) != 0x3F);

	printf("Successful calibration of all BNO055 Orientation Sensor components.");

}


/*
 * --------------------------------------------------------------------------------------------------------
 * FUNCTION: get_imu_data
 * --------------------------------------------------------------------------------------------------------
 *
 * Sends the command to read the Euler Angle data of the BNO055 Orientation Sensor and stores the data
 * in a buffer defined in the calling function.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * PARAMETERS:
 *     UART_IMU_ID:         The device ID of the BNO055 sensor on UART.
 *
 *     imuBuffer:           Pointer to a buffer in which to store the data read from the sensor.
 *
 *     uartErrorLog:        Pointer to an array for storing error codes.
 *
 *     uartErrorLogCounter: Pointer to a counter to keep track of the number of error codes recorded.
 *
 * --------------------------------------------------------------------------------------------------------
 *
 * RETURNS: Returns 0 if the process is successful; -1 if an error occurs. See uartErrorLog for details.
 *
 * --------------------------------------------------------------------------------------------------------
 */
int get_imu_data(const int UART_IMU_ID, uint8_t *imuBuffer, int8_t *uartErrorLog, size_t *uartErrorLogCounter)
{
	uint8_t numberOfBytes = 0;
	const int imuCmdLength = 4;
	/* Define "imuCmd" -- the command to send to the BNO055 on UART to read Euler data.
	 * BYTE0: Initial byte for any UART command.
	 * BYTE1: Value of 0x01 means this is a READ command.
	 * BYTE2: Start Reg Address: 8-bit registers 0x1A through 0x1F hold 3 16-bit values for Euler Angles
	 * BYTE3: Number of Bytes of Data: Read 6 8-bit registers 0x1A through 0x1F
	 */
	uint8_t imuCmd[imuCmdLength] = {0xAA, 0x01, 0x1A, 0x06};
	const int IMU_RESPONSE_LENGTH = 2;
	uint8_t imuResponse[IMU_RESPONSE_LENGTH] = {0x00, 0x00};
	int imu_return_value;
	
	imu_return_value = uart_send(UART_IMU_ID, imuCmd, imuCmdLength, imuResponse, IMU_RESPONSE_LENGTH, uartErrorLog, uartErrorLogCounter);
	
	if(imu_return_value == -1)
	{
		return -1;
	}
	else if (imu_return_value == 0 && ((numberOfBytes = imuResponse[1]) == 6) )
	{
		int read_return_value = read(UART_IMU_ID, imuBuffer, numberOfBytes);
		if(read_return_value == -1)	// Error attempting read command
		{
			uartErrorLog[(*uartErrorLogCounter)++] = UART_READ_ERROR;
			return -1;
		}
		else if(read_return_value == numberOfBytes)	// Successful read of 6 bytes of data
		{
			return 0;
		}
		else
		{
			uartErrorLog[(*uartErrorLogCounter)++] = UART_INCOMPLETE_READ;
			return -1;
		}
	}
	else
	{
		uartErrorLog[(*uartErrorLogCounter)++] = UART_SEND_ERROR;
		return -1;
	}
}
