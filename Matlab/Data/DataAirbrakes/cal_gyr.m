function [ gx, gy, gz ] = cal_gyr( gx, gy, gz, t0_rest, t1_rest, timeMillis)
%CAL_GYR Summary of this function goes here
% input when the system is still 16.4lsb/deg/sec
% the mean value of gyro during a rest period [t0 t1] is removed from data
% unbiased output in rad/sec

% build the rest_period
rest_period = timeMillis>t0_rest & timeMillis<t1_rest ;

% gyro calibration
gx_offset = mean(gx(rest_period));
gy_offset = mean(gy(rest_period));
gz_offset = mean(gz(rest_period));
gx = (gx-gx_offset)*pi/(16.4*180);
gy = (gy-gy_offset)*pi/(16.4*180);
gz = (gz-gz_offset)*pi/(16.4*180);

end

