# Name:    Orientation.py
# Date:    April 22, 2018
# Project: Flight Data Collection System
# Author:  Martin DeWitt, Assistant Professor of Physics, High Point University
#
# Purpose: Correct pitch angle from BNO055 data collected. Generate plots of
#          Heading vs. Time, Roll vs. Time, and Pitch vs. Time.

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

# Read in BNO055 orientation sensor data
df_imu = pd.read_csv("sample_data/imudata.csv")

# Reference time to first data point
df_imu.loc[:, "time(s)"] = df_imu.loc[:, "time(s)"] - df_imu.loc[0, "time(s)"]

# Adjust pitch angle due to orientation of sensor
criteria = df_imu.loc[:, "pitch(deg)"] > 0
df_imu.loc[criteria, "pitch(deg)"] = df_imu.loc[criteria, "pitch(deg)"] - 180.0
df_imu.loc[~criteria, "pitch(deg)"] = df_imu.loc[~criteria, "pitch(deg)"] + 180.0

# Select the range of times that were during actual flight
MIN_TIME = 215		# Takeoff occurs at about 215 seconds after the start of data recording
MAX_TIME = 720		# Landing occurs at about 720 seconds after the start of data recording
s_flight_times_bool = df_imu["time(s)"].between(MIN_TIME, MAX_TIME)

# Print part of the final dataframe
print(df_imu.loc[s_flight_times_bool, :].head())

# Generate plots
plt.plot(df_imu.loc[s_flight_times_bool, "time(s)"], df_imu.loc[s_flight_times_bool, "heading(deg)"])
plt.title("Compass Heading vs. Time")
plt.xlabel("Time (s)")
plt.ylabel("Heading (deg)")
plt.show()

plt.plot(df_imu.loc[s_flight_times_bool, "time(s)"], df_imu.loc[s_flight_times_bool, "roll(deg)"])
plt.title("Roll Angle vs. Time")
plt.xlabel("Time (s)")
plt.ylabel("Roll Angle (deg)")
plt.show()

plt.plot(df_imu.loc[s_flight_times_bool, "time(s)"], df_imu.loc[s_flight_times_bool, "pitch(deg)"])
plt.title('Pitch Angle vs. Time')
plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (deg)')
plt.show()