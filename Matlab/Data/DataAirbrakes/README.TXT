{\rtf1\ansi\ansicpg1252\cocoartf1561\cocoasubrtf100
{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\paperw11900\paperh16840\margl1440\margr1440\vieww7740\viewh18000\viewkind0
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0

\f0\fs24 \cf0 Important files in DataAirbrakes:\
\

\b - Kaltbrunn18_11_17_original.TXT
\b0 \
Contains the raw data + events of Kaltrbrunn flight\
\
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0

\b \cf0 - Kaltbrunn18_11_17.TXT
\b0 \
Contains the raw data without any text of Kaltrbrunn flight\
\

\b - Kaltbrunn18_11_17.csv
\b0 \
Conversion in csv of the altbrunn18_11_17.TXT to be readable by Matlab\
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0
\cf0 \

\b - dataAnalysisKaltrbunn.m
\b0 \
load the flight data\
Post-processing:\
	integrate acceleration to have smooth speed and altitude\
	convert barometer data into altitude\
	integrate gyro data to obtain angles Pitch and Yaw\
	compute inclination of rocket from Pitch and Yaw\
	find initial orientation of the rocket on launchpad\
	calibrate gyro and magnetometer\
	compute velocity from pitot sensor\
	identify drag coefficient of the rocket when the airbrakes are open/closed\
Plots all these information}