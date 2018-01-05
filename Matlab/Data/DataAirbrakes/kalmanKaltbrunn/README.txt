This files perform a Kalman Filter test on the data of the Kaltbrunn launchday
inputs: sensors data, motion model
output: state estimates of the rocket (altitude in Earth's frame, 
		velocity and acceleration in rocket's frame)


Important files in kalmanKaltbrunn:

-main.m
initialize the variables and matrices of Kalman Filter
call the function Kalman.m and plot the results

-kalman.m
defines the class kalman (containing all necessary varibales)
call a function that update the states (alt, speed, acc) and the matrices

-motion_model.m
contains the equation of motion of the rocket
called by kalman.m to update the states at each iteration 