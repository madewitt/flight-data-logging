# Flight Data Collection System

![](images/exterior.jpg)

This repository houses code for a flight data collection system developed for an 
unmanned aerial vehicle. This includes code used to collect data, as well as
code used to process, clean, and visualize the data.
 [Click here](https://madewitt.com/2019/02/13/flight-data-collection-system/) for more
details about the project.

---

## What data are collected?
![](images/interior.jpg)

A Raspberry Pi is used as an onboard computer. Roll, pitch, and heading are measured with
an Adafruit BNO055 orientation sensor. Airspeed and angle of attack are measured using a
system of two pitot-static tubes connected to differential pressure sensors. One pitot-static
tube is mounted under the right wing facing toward the front of the aircraft, while the other
is mounted under the left wing and is tilted downward at an 80 degree angle. Elevator deflection
and throttle setting are measured by connecting the PWM outputs of aircraft's radio receiver
to GPIO pins on the Pi. Using interrupts, the rising and falling edges of the PWM signal are
recorded so that the PWM signal can be reconstructed. Calibration functions are used to convert
the raw collected data into final values.

---

## Summary of the Code
Code written in C is used by the Raspberry Pi to interact with the sensors and log data. Python
code uses the `pandas` library to read and manipulate data, the `scipy` library to filter noise
and perform numerical root finding, and the `matplotlib` library to generate plots. The files in
the repository include:

### C Code for Data Collection
- `collection/FlightSensors.h` and `collection/FlightSensors.c`
	- Contains functions and interrupt service routines used to communicate with sensors.
- `collection/DataCollection.c`
	- Initialize and calibrate sensors, setup interrupts, poll orientation sensor, and write data
	  to CSV files.
	  
### Python Code for Data Processing and Visualiztion
- `processing/AirspeedAndAOA.py`
	- Smooth out noise in the pressure data using a Savitzky-Golay filter.
	- Compute airspeed and angle of attack are from calibration functions.
	- Generate plots of Airspeed vs. Time and Angle of Attack vs. Time. 
- `processing/Orientation.py`
	- Correct pitch angle.
	- Generate plots of Roll vs. Time, Pitch vs. Time, and Heading vs. Time.
- `processing/Controls.py`
	- Compute the "ON TIME" of the PWM signal from rising edge and falling edge times.
	- Compute elevator deflection angle and static thrust from PWM "ON TIME."
	- Generate plots of Elevator Deflection vs. Time and Static Thrust vs. Time.

---

## Compiling and Running the Data Collection Code
The data collection code assumes that the sensors are connected to the 
Raspberry Pi pins as follows:

- SPI (DLHR-L10D differntial pressure sensor)
	- a
	- b
- I<sup>2</sup>C (DLHR-L10D differntial pressure sensor)
	- a
	- b
- UART (Adafruit BNO055 orientation sensor)
	- a
	- b
- Interrupt Pins
	- DLHR-L10D pressure sensor (SPI) interrupt pin connected to (#11)--GPIO17 which is WiringPi Pin 0
	- DLHR-L10D pressure sensor (I2C) interrupt pin connected to (#12)--GPIO18 --> WiringPi Pin 1
	- Elevator PWM signal from receiver connected to (#13)--GPIO27 --> WiringPi Pin 2
	- Throttle PWM signal from receiver connected to (#15)--GPIO22 --> WiringPi Pin 3

The data collection code uses the [WiringPi library](http://wiringpi.com/).
WiringPi is now pre-installed on standard Raspbian systems. The pin
numbers referenced in the code are
[WiringPi pin numbers](http://wiringpi.com/pins/), not the
standard Raspberry Pi pin numbers.

1. Follow the directions
[here](http://wiringpi.com/download-and-install/) to make sure it is pre-installed.
If not, follow directions to download and install.
2. In the `collection` folder, compile DataCollection.c, making sure to link the wiringPi
library. For example:
```
gcc -Wall -o DataCollection DataCollection.c -lwiringPi
```
3. 