# ############################################################################################################################
#
#	Name: 		Airspeed&AOA.py
#	Date: 		April 22, 2018
#	Project:	Flight Data Collection System
#	Author:		Martin DeWitt, Assistant Professor of Physics, High Point University
#
#   Purpose: 	Convert data collected from differential pressure sensors connected to
#				pitot-static tubes into airspeed and angle of attack and output plots of
#				Airspeed vs. Time and Angle of Attack vs. Time.
#
#	Method:		Logged data stored in CSV files is read into Pandas DataFrames. There are
#				four sets read in:
#					1) Background data collected for forward facing pitot-static tube at rest indoors (i.e. no wind).
#					2) Background data collected for tilted pitot-static tube at rest indoors (i.e. no wind).
#					3) Flight data collected for forward-facing pitot-static tube.
#					4) Flight data collected for tilted pitot-static tube.
#				The background measurements for each pitot-static tube are averaged separately to produce mean
#				background pressure readings. The mean backgound measurement for each pitot-static tube is
#				subtracted from the in-flight measurements producing two sets of corrected differential pressure
#				measurements: dP1 and dP2.  A Savitzky-Golay filter is used to smooth out noise in the signals.
#				
#				Interrupts were used to know when the measurements were ready on each sensor. Therefore, the measurement
#				times for the two sensors are not the same. Linear interpolation is used to estimate the value of dP1 for
#				the times at which the dP2 measurements are recorded. This approximates having dP1 and dP2 measured at
#				the same time. This is necessary since the determination of angle of attack requires knowing both
#				measurements simultaneously.
#
#				Previously determined calibration functions are used to calculate airspeed and angle of attack. For
#				details regarding how these functions were obtained, please see the following webpage:
#				https://madewitt.com/2019/02/13/flight-data-collection-system/
#
#				The airspeed calibration function is a simple quadratic equation, where dP is the reading from the forward
#				pitot-static probe (dP1) and V is the airspeed.
#						Airspeed calibration function:  dP = (0.679)V^2 + (0.308)V - 0.718
#						Regrouping: ax^2 + bx + c = 0   where a = 0.679, b = 0.308, c = (-0.718 - dP)
#				For all physical values of dP, the calibration function has two real roots: one positive and one negative.
#				The positive root, [(-b + sqrt(b^2 - 4ac))/2a], found using the quadratic formula is the physically
#				possible airspeed.
#
#				The angle of attack calibration function is more complex and requires a numerical method to solve.
#				The pressure ratio (dP1/dP2) as a function of angle of attack (x) in radians is given by:
#						(dP1/dP2) = [E*x^3 + F*x^2 + G*x + H] / [A*cos(B*(x + shift) + C) + D]
#				where A - H are calibration constants and shift is the angle in radians between the two pitot-static
#				tubes. Calling the numerator slopeFunc1(x) and the denominator slopeFunc2(x) and rearranging gives:
#						(slopeFunc1(x)/slopeFunc2(x)) - (dP1/dP2) = 0
#				For a given value of the pressure ratio, the method scipy.optimize.root is used to find the root x
#				of this last equation, which is the angle of attack.
#
#				Finally, matplotlib.pyplot is used to generate plots of Airspeed vs. Time and Angle of Attack vs. Time.
#
# #############################################################################################################################

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

def getAirspeed(dP):
	"""Calculate airspeed from given differential pressure measurement(s) using calibration function.
	
	Parameters:
		dP (float, ndarray, Series): differential pressure measurement(s) in Pascals from pitot-static tube
	
	Returns:
		(float, ndarray, Series): airspeed in meters per second
	"""
	# Calibration constants
	a = 0.679
	b = 0.308
	c = -0.718 - dP
	# Quadratic formula
	numerator = -b + np.sqrt(b**2 - 4*a*c)
	denominator = 2*a
	return np.divide(numerator, denominator)
	


def angleOfAttackFunction(x, pressure_ratio, shift=np.radians(80.0)):
	"""Calculate the value of the angle of attack calibration function given an angle of attack, x, and
	a ratio of pressure measurements(pressure_ratio = dP1/dP2).
	
	Parameters:
		x (float): angle of attack in radians
		pressure_ratio (float): ratio of the pitot static tube measurements (dP1/dP2)
		shift (float): angle between the two pitot-static tubes in radians
	
	Returns:
		(float): value of the calibration function (NOTE: value is zero if x is a root)
	"""
	slopeFunc2 = 1.46641809*np.cos(1.77632305*(x + shift) - 0.16632905) - 0.46524997
	x = np.abs(x)	# Makes slopeFunc1 symmetric about x=0 (i.e. same whether forward probe tilts upward or downward)
	slopeFunc1 = -2.28525369*x**3.0 - 0.37086941*x**2.0 - 0.02234251*x + 1.0019943   
	return (slopeFunc1/slopeFunc2 - pressure_ratio)
	

# Read in background data (taken with pitot-static tubes at rest with no wind)
df_background1 = pd.read_csv(r"sample_data\background\dp1data.csv")		# DataFrame: background values (time, dP1)
df_background2 = pd.read_csv(r"sample_data\background\dp2data.csv")		# DataFrame: background values (time, dP2)

# Find the means of the background data
s_bg_mean1 = df_background1.mean(axis=0)	# Series: mean values of time and background pressure differential dP1
s_bg_mean2 = df_background2.mean(axis=0)	# Series: mean values of time and background pressure differential dP2
dP1_bg_mean = s_bg_mean1.loc['dP1(Pa)']		# Float: Mean value of background dP1
dP2_bg_mean = s_bg_mean2.loc['dP2(Pa)']		# Float: Mean value of background dP2

# Read in flight data files dp1data (time1, dP1) and dp2data (time2, dP2). NOTE: times are not the same for the two files
df_dP1 = pd.read_csv("sample_data/flight_data/dp1data.csv")	# DataFrame: Pitot tube #1 data from flight (time1, dP1)
df_dP2 = pd.read_csv("sample_data/flight_data/dp2data.csv")	# DataFrame: Pitot tube #2 data from flight (time2, dP2)

# Subtract background from both data sets
df_dP1.loc[:, 'dP1_bs(Pa)'] = df_dP1.loc[:, 'dP1(Pa)'] - dP1_bg_mean	# Create new column with background pressure reading subtracted
df_dP2.loc[:, 'dP2_bs(Pa)'] = df_dP2.loc[:, 'dP2(Pa)'] - dP2_bg_mean	# Create new column with background pressure reading subtracted

# Use Savitzky-Golay to smooth dP1 and dP2 data
from scipy.signal import savgol_filter
WINDOW_SIZE = 31
ORDER = 2
df_dP1.loc[:, 'dP1_filt(Pa)'] = savgol_filter(x=df_dP1.loc[:,'dP1_bs(Pa)'], window_length=WINDOW_SIZE, polyorder=ORDER)
df_dP2.loc[:, 'dP2_filt(Pa)'] = savgol_filter(x=df_dP2.loc[:,'dP2_bs(Pa)'], window_length=WINDOW_SIZE, polyorder=ORDER)
 
# Use linear interpolation (scipy.interpolate.interp1d) to estimate the value of dP1 at the times of dP2 measurements
from scipy.interpolate import interp1d	# interp1d returns an interpolation function y(x) so that y can be evaluated at any value of x
dP1_interp = interp1d(x=df_dP1.loc[:, 'time(s)'], y=df_dP1.loc[:, 'dP1_filt(Pa)'], fill_value='extrapolate')	# Interpolation function y(x) = dP1(time)
df_dP2.loc[:, 'dP1_interp(Pa)'] = dP1_interp(df_dP2.loc[:, 'time(s)'])	# DataFrame: Create new column with values of dP1 at times of dP2

# Subtract the intial time from all times so that time begins at 0 seconds
df_dP2.loc[:, 'time_zero(s)'] = df_dP2['time(s)'] - df_dP2.loc[0, 'time(s)']	# DataFrame: Create new column with time referenced to initial time

# Create a new DataFrame with transformed data and simplify labels
df_Data = df_dP2.loc[:, ('time_zero(s)', 'dP1_interp(Pa)', 'dP2_filt(Pa)')]
df_Data.rename(columns={'time_zero(s)':'time(s)', 'dP1_interp(Pa)':'dP1(Pa)', 'dP2_filt(Pa)':'dP2(Pa)'}, inplace=True)

# Calculate airspeed using dP1 (from forward facing pitot-static tube)
df_Data.loc[:, 'V(m/s)'] = df_Data.loc[:, 'dP1(Pa)'].apply(getAirspeed)

# Determine angle of attack (alpha) by finding the roots of a calibration function (angleOfAttackFunction) that uses the ratio dP1/dP2
from scipy.optimize import root
df_Data.loc[:, 'dP1/dP2'] = df_Data.loc[:, 'dP1(Pa)'].divide(df_Data.loc[:, 'dP2(Pa)'])	# DataFrame: add new column with ratio of pressure readings
pressure_ratio = df_Data.loc[:, 'dP1/dP2'].values	# ndarray: array of values of the pressure ratio dP1/dP2
alpha = np.zeros(pressure_ratio.size)		# ndarray: array to hold calculated values of angle of attack (alpha); initialized to zero
print("Calculating. Please wait...")
for row in np.arange(0, pressure_ratio.size):
	print("Counting down:", int(25 - row/1000)) if row%1000 == 0 else 0
	# The 'x' attribute of the OptimizeResult returned by root() gives the value of the obtained root
	alpha[row] = np.degrees(-1.0*root(angleOfAttackFunction, 0, args=(pressure_ratio[row])).x)

df_Data.loc[:, 'alpha(deg)'] = alpha;	# DataFrame: Add the values of angle of attack in a new column

# Output final data table and plots
# Select the range of times that were during actual flight
MIN_TIME = 215	# Takeoff occurs at about 215 seconds after the start of data recording
MAX_TIME = 720	# Landing occurs at about 720 seconds after the start of data recording
s_flight_times_bool = df_Data['time(s)'].between(MIN_TIME, MAX_TIME)	# Series: Boolean criteria indicates where MIN_TIME <= time <= MAX_TIME

# Sample of final data
print(df_Data.loc[s_flight_times_bool, :].head())

# Plots of the airspeed and angle of attack during flight
plt.plot( df_Data.loc[s_flight_times_bool, 'time(s)'], df_Data.loc[s_flight_times_bool, 'V(m/s)'] )
plt.title("Airspeed vs. Time")
plt.xlabel("Time (s)")
plt.ylabel("V (m/s)")
plt.show()

plt.plot( df_Data.loc[s_flight_times_bool, 'time(s)'], df_Data.loc[s_flight_times_bool, 'alpha(deg)'] )
plt.title("Angle of Attack vs. Time")
plt.xlabel("Time (s)")
plt.ylabel(r"$\alpha$ (deg)")
plt.show()