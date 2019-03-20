# Name:    Controls.py
# Date:    April 22, 2018
# Project: Flight Data Collection System
# Author:  Martin DeWitt, Assistant Professor of Physics, High Point University
#
# Purpose: Convert rising and falling edge times of elevator servo and electronic
#          speed controller PWM signals into elevator deflection angle and
#          static thrust. Generate plots of Elevator Deflection vs. Time and
#          Static Thrust vs. Time.
#
# Method: Calibration functions for the elevator deflection angle and static thrust
#         can be found in the project writeup here:
#            https://madewitt.com/2019/02/13/flight-data-collection-system/
#         Note that the time in those calibration equations is in microseconds.
#         Therefore the functions in this script convert seconds to microseconds.

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

def elevatorCalibration(t_on):
	"""Calculate elevator deflection angle in degrees.

	Parameters:
	   t_on (float, ndarray, Series): ON TIME of the elevator PWM signal in seconds
	
	Returns: (float, ndarray, Series): Elevator deflection angle in degrees; positive
	         deflection is downward
	"""
	return np.degrees(7.403e-4 * (t_on * 1.0e6) - 1.110)
	
def staticThrustCalibration(t_on):
	"""Calculate static thrust in newtons.

	Parameters:
	   t_on (float, ndarray, Series): ON TIME of the throttle pwm signal in seconds
	
	Returns: (float, ndarray, Series): Static thrust in newtons.
	"""
	return 1.58e-2 * (t_on * 1.0e6) - 18.94

# Read in rising and falling edge time data for elevator servo and electronic speed controller.
df_elevator_raw = pd.read_csv("sample_data/elevatordata.csv")
df_throttle_raw = pd.read_csv("sample_data/throttledata.csv")

# Reference time to beginning of elevator data
time_ref = df_elevator_raw.loc[0, 'time(s)']	
df_elevator_raw.loc[:, 'time(s)'] = df_elevator_raw.loc[:, 'time(s)'] - time_ref
df_throttle_raw.loc[:, 'time(s)'] = df_throttle_raw.loc[:, 'time(s)'] - time_ref

# Take the difference between successive times.
# Differences will alternate between the PWM signal's ON TIME and OFF TIME.
df_elevator_raw.loc[:, 't_diff'] = df_elevator_raw.diff(periods=1, axis=0)
df_throttle_raw.loc[:, 't_diff'] = df_throttle_raw.diff(periods=1, axis=0)

# ON TIME is between 1000 microseconds and 2200 microseconds. Select only these rows and generate new dataframes.
PWM_MIN = 0.001000
PWM_MAX = 0.002200
elevator_criteria = df_elevator_raw.loc[:, 't_diff'].between(PWM_MIN, PWM_MAX)
throttle_criteria = df_throttle_raw.loc[:, 't_diff'].between(PWM_MIN, PWM_MAX)

df_elevator = pd.DataFrame(data=df_elevator_raw.loc[elevator_criteria, :].values, columns=['time(s)', 't_on(s)']) 
df_throttle = pd.DataFrame(data=df_throttle_raw.loc[throttle_criteria, :].values, columns=['time(s)', 't_on(s)']) 

# Use calibration functions to calculate elevator deflection angle and static thrust.
df_elevator.loc[:, 'delta_e_unfilt(deg)'] = df_elevator.loc[:, 't_on(s)'].apply(func=elevatorCalibration)
df_throttle.loc[:, 'static_thrust_unfilt(N)'] = df_throttle.loc[:, 't_on(s)'].apply(func=staticThrustCalibration)

# Use Savitzky-Golay to smooth noise
from scipy.signal import savgol_filter
WINDOW_SIZE = 45
ORDER = 2
df_elevator.loc[:, 'delta_e(deg)'] = savgol_filter(x=df_elevator.loc[:,'delta_e_unfilt(deg)'], window_length=WINDOW_SIZE, polyorder=ORDER)
df_throttle.loc[:, 'static_thrust(N)'] = savgol_filter(x=df_throttle.loc[:,'static_thrust_unfilt(N)'], window_length=WINDOW_SIZE, polyorder=ORDER)


# Output final data table and plots
# Select the range of times that were during actual flight
MIN_TIME = 215	# Takeoff occurs at about 215 seconds after the start of data recording
MAX_TIME = 720	# Landing occurs at about 720 seconds after the start of data recording
elevator_time_criteria = df_elevator['time(s)'].between(MIN_TIME, MAX_TIME)
throttle_time_criteria = df_throttle['time(s)'].between(MIN_TIME, MAX_TIME)

print(df_elevator.loc[elevator_time_criteria, :].head())
print(df_throttle.loc[throttle_time_criteria, :].head())

plt.plot(df_elevator.loc[elevator_time_criteria, 'time(s)'],
		df_elevator.loc[elevator_time_criteria, 'delta_e_unfilt(deg)'])
plt.plot(df_elevator.loc[elevator_time_criteria, 'time(s)'],
		df_elevator.loc[elevator_time_criteria, 'delta_e(deg)'])
plt.title("Elevator Deflection vs. Time")
plt.xlabel("Time(s)")
plt.ylabel(r"$\delta_e$ (deg)")
plt.legend()
plt.show()

plt.plot(df_throttle.loc[throttle_time_criteria, 'time(s)'],
		df_throttle.loc[throttle_time_criteria, 'static_thrust_unfilt(N)'])
plt.plot(df_throttle.loc[throttle_time_criteria, 'time(s)'],
		df_throttle.loc[throttle_time_criteria, 'static_thrust(N)'])
plt.title("Static Thrust vs. Time")
plt.xlabel("Time(s)")
plt.ylabel("Static Thrust (N)")
plt.legend()
plt.show()