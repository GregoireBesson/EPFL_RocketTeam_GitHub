function [ gx, gy, gz ] = cal_gyr( gx, gy, gz )
%CAL_GYR Summary of this function goes here
% input when the system is still 16.4lsb/deg/sec
% unbiased output in rad/sec

% gyro calibration

gx_offset = mean(gx);
gy_offset = mean(gy);
gz_offset = mean(gz);
gx = (gx-gx_offset)*pi/(16.4*180);
gy = (gy-gy_offset)*pi/(16.4*180);
gz = (gz-gz_offset)*pi/(16.4*180);

end

