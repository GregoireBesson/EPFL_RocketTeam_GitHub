Important files in DataAirbrakes:

- Kaltbrunn18_11_17_original.TXT
Contains the raw data + events of Kaltrbrunn flight

- Kaltbrunn18_11_17.TXT
Contains the raw data without any text of Kaltrbrunn flight

- Kaltbrunn18_11_17.csv
Conversion in csv of the altbrunn18_11_17.TXT to be readable by Matlab

- dataAnalysisKaltrbunn.m
load the flight data
Post-processing:
	integrate acceleration to have smooth speed and altitude
	convert barometer data into altitude
	integrate gyro data to obtain angles Pitch and Yaw
	compute inclination of rocket from Pitch and Yaw
	find initial orientation of the rocket on launchpad
	calibrate gyro and magnetometer
	compute velocity from pitot sensor
	identify drag coefficient of the rocket when the airbrakes are open/closed
Plots all these information