function [ gx, gy, gz ] = cal_gyr( gx, gy, gz, t0_rest, t1_rest, timeMillis)
%CAL_GYR Summary of this function goes here
% input when the system is still 16.4lsb/deg/sec
% unbiased output in rad/sec
% the mean value og gyro during a rest period [t0 t1] is removed from data

% gyro calibration
rest_period = timeMillis>t0_rest & timeMillis<t1_rest ;


gx_offset = mean(gx(rest_period));
gy_offset = mean(gy(rest_period));
gz_offset = mean(gz(rest_period));
gx = (gx-gx_offset)*pi/(16.4*180);
gy = (gy-gy_offset)*pi/(16.4*180);
gz = (gz-gz_offset)*pi/(16.4*180);

end

