% Data Analysis of a rocket test flight
% Sensors: 6DoF IMU, Magnetometer, Barometer
% Author: Grégoire Besson / Remi Baudoux
% Date: 19/11/2017

%% Initialization

clc
clear all
close all
firstRun = true;

%% Load the data

filename = 'Kaltbrunn18_11_17.csv';
% row offset
R1 = 2;
% column offset
C1 = 0;
M = csvread(filename,R1,C1);

timeMicros = M(:,1);
ax = M(:,2);
ay = M(:,3);
az = M(:,4);
gx = M(:,5);
gy = M(:,6);
gz = M(:,7);
mx = M(:,8);
my = M(:,9);
mz = M(:,10);
press = M(:,11)/100;    % pressure is read in Pa, converted to hPa
deltaP = M(:,12);

%% Processing

% time offset (motor starts burning at t=0ms)
tOffset = 5.55e8-93.17;
t0 = 0;

% events
tm = 554972368 - tOffset;
topen1 = 557485792 - tOffset;
tclose1 = 558291264 - tOffset;
topen2 = 559097208 - tOffset;
tclose2 = 559907908 - tOffset;
topen3 = 560721528 - tOffset;
tclose3 = 561529152 - tOffset;
topen4 = 562343100 - tOffset;
tBurnout = 0;
tApogee = 7541.7;
tPara = 1.033e4;

timeMicros = timeMicros - tOffset;
timeMillis = timeMicros/1000;
timeSec = timeMillis/1000;
timeMin = timeSec/60;
minMillis = -1000;
maxMillis = 40000;
minSec = minMillis/1000;
maxSec = maxMillis/1000;

% acceleration from LSBm/s^2 to m/s^2
accFactorUnits = 1/417.5;
axMS2 =  ax*accFactorUnits;
ayMS2 = -ay*accFactorUnits;
azMS2 =  az*accFactorUnits;
% convertion in g
axG = axMS2/9.81; 
ayG = ayMS2/9.81;
azG = azMS2/9.81;

%gyroscope calibraiton
%[gx, gy, gz] = cal_gyr(gx, gy, gz);
%removal of aberrant values on the first run
% if (firstRun)
%     k = find(press>950);
%     press(k) = press(k(1)-1);
%     firstRun = false;
% end

% compute the altitude from the barometer
seaLevelhPA = 1013.25;
alt = 44330*(1-((press)/seaLevelhPA).^(0.1903));
alt = alt - alt(1);

% compute the speed from the pitot sensor
deltaPoffset = deltaP(10);
deltaP = deltaP - deltaPoffset;
% PRESSURE_SENSOR2_MAX = 103421/6;
% PRESSURE_SENSOR2_MIN = 0;
% deltaP_ranged = (deltaP - deltaPoffset)*...
%                 (PRESSURE_SENSOR2_MAX - PRESSURE_SENSOR2_MIN) / ...
%                 (max(deltaP) - deltaPoffset) + PRESSURE_SENSOR2_MIN;
RHO_AIR = 1.225;
velocityPitot = sqrt(abs(2 * deltaP / RHO_AIR)); 

%% Plots of pressure and altitude

displayInfos = false;

figure(1)
title('Data from Baromoter')

subplot(2,1,1)
plot(timeMillis,press,'Linewidth',1.5)
ylabel('Pressure [hPa]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(2,1,2)
plot(timeMillis,alt,'Linewidth',1.5)
ylabel('Altitude [m]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

%% Plots of magnetometers on three axis

figure(2)
%magnetometer calibration
[mx, my, mz] = cal_mag(mx, my, mz);

figure(3)
subplot(3,1,1)
plot(timeMillis,mx,'Linewidth',1.5)
ylabel('m_x [\muT]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
title('Magnetic fields')
set(gca,'fontsize', 16);
grid on

subplot(3,1,2)
plot(timeMillis,my,'Linewidth',1.5)
ylabel('m_y [\muT]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(3,1,3)
plot(timeMillis,mz,'Linewidth',1.5)
ylabel('m_z [\muT]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

%% Plots of gyro on three axis

figure(4)
subplot(3,1,1)
plot(timeMillis,gx,'Linewidth',1.5)
ylabel('g_x [°/s]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
title('Angular rates')
set(gca,'fontsize', 16);
grid on

subplot(3,1,2)
plot(timeMillis,gy,'Linewidth',1.5)
ylabel('g_y [°/s]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(3,1,3)
plot(timeMillis,gz,'Linewidth',1.5)
ylabel('g_z [°/s]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

%% Plots of acceleration on three axis

figure(5)
subplot(3,1,1)
plot(timeMillis,axG,'Linewidth',1.5)
ylabel('a_x [g]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
title('Accelarations')
set(gca,'fontsize', 16);
grid on

subplot(3,1,2)
plot(timeMillis,ayG,'Linewidth',1.5)
ylabel('a_y (Normal) [g]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(3,1,3)
plot(timeMillis,azG,'Linewidth',1.5)
ylabel('a_z(Radial) [g]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

%% Relevant Plots

figure(6)
subplot(4,1,1)
plot(timeMillis,ayG,'Linewidth',1.5)
ylabel('a_y [g]')
xlabel('Time [ms]');
xlim([minMillis tApogee+1500]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(4,1,2)
plot(timeMillis,press,'Linewidth',1.5)
xlim([minMillis tApogee+1500]);
ylabel('Pressure [hPa]')
xlabel('Time [ms]');
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(4,1,3)
plot(timeMillis,alt,'Linewidth',1.5)
xlim([minMillis tApogee+1500]);
ylabel('Alttitude [m]')
xlabel('Time [ms]');
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(4,1,4)
plot(timeMillis,velocityPitot,'Linewidth',1.5)
xlim([minMillis tApogee+1500]);
ylabel('Veloctiy from Pitot [m/s]')
xlabel('Time [ms]');
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

%% Accelration versus Airbrakes signal

AirbrakesSignal = ones(size(ay));
AirbrakesSignal = - AirbrakesSignal;
BrakeIndexes = find(timeMicros>=topen1 & timeMicros<tclose1 | ... 
                    timeMicros>=topen2 & timeMicros<tclose2 | ... 
                    timeMicros>=topen3 & timeMicros<tclose3 | ... 
                    timeMicros>=topen4 );
AirbrakesSignal(BrakeIndexes) = 1;

figure(7)
%yyaxis left
plot(timeMillis,ayG,'Linewidth',1.5,'DisplayName','Acceleration')
ylabel('Magnitude [g, -]')
xlabel('Time [ms]');
xlim([minMillis tApogee+1500]);
%ylim([-50 100]);
set(gca,'fontsize', 16);
grid on
hold on
%yyaxis right
plot(timeMillis, AirbrakesSignal, 'Linewidth',1.5,'DisplayName','Airbrakes Signal')
legend show
%ylabel('Airbrakes Signal')
%ylim([-1 10]);