/*
 *	Title: 		FlightSensors.h
 *	Project:	Flight Data Collection System
 *	Author: 	Martin A. DeWitt, Assistant Professor of Physics, High Point University
 *	Date: 		April 15, 2018
 *  
 *	Purpose:	Functions and interrupt service routines used to communicate with sensors.
 *
 */
#ifndef _FLIGHT_SENSORS_H
#define _FLIGHT_SENSORS_H

#include<stdint.h>

void elevator_isr(void);
void throttle_isr(void);
int request_new_spi_measurement(void);
int get_spi_pressure_data(uint8_t *buffer, size_t bufferLength);
void spi_pressure_isr(void);
int request_new_i2c_measurement(void);
void i2c_pressure_isr(void);
int8_t uart_send(int8_t uartId, uint8_t *uartCmd, int8_t uartCmdLength, uint8_t *uartResponse, int8_t uartResponseLength, int8_t *uartErrorLog, size_t *uartErrorLogCounter);
void start_imu_fusion_mode(const int UART_IMU_ID, int8_t *uartErrorLog, size_t *uartErrorLogCounter);
void calibrate_imu_sensors(const int UART_IMU_ID, int8_t *uartErrorLog, size_t *uartErrorLogCounter);
int8_t get_imu_data(const int UART_IMU_ID, uint8_t *imuCmd, uint8_t *uartResponse, uint8_t *imuBuffer);

#endif