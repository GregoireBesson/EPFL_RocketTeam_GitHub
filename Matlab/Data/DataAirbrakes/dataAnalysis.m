% Data Analysis of a rocket test flight
% Sensors: 6DoF IMU, Magnetometer, Barometer
% Author: Gr�goire Besson
% Date: 09/10/2017

%% Initialization

clc
clear all
close all
firstRun = true;

%% Load the data

filename = 'dataTestFlight07_10_17.csv';
% row offset
R1 = 832;
% column offset
C1 = 0;
M = csvread(filename,R1,C1);

timeMillis = M(:,1);
ax = M(:,2);
ay = M(:,3);
az = M(:,4);
gx = M(:,5);
gy = M(:,6);
gz = M(:,7);
mx = M(:,8);
my = M(:,9);
mz = M(:,10);
temp = M(:,11);
% pressure is read in Pa, converted to hPa
press = M(:,12)/100;

%% Processing

% time offset (motor starts burning at t=0ms)
tOffset = 1.820634e6;
t0 = 0;
% obtained graphically, coherent with expected burnout time
tBurnout = 1975;
% condition1 is t(a_y < 20g)+500ms  
tABcond1 = 2375;
% condition2 is activated 2200ms after the motor starts burning
tABcond2 = t0 + 2200;
tPara = 9198;

timeMillis = timeMillis - tOffset;
timeSec = timeMillis/1000;
timeMin = timeSec/60;
minMillis = -1000;
maxMillis = 15000;
minSec = minMillis/1000;
maxSec = maxMillis/1000;

% radial acceleration from ? to m/s^2
accFactorUnits = 1/208.77;
axG =  ax*accFactorUnits;
ayG = -ay*accFactorUnits;
azG =  az*accFactorUnits;

%magnetometer calibration
[mx, my, mz] = cal_mag(mx, my, mz);
%gyroscope calibraiton
[gx, gy, gz] = cal_gyr(gx, gy, gz);
%removal of aberrant values on the first run
if (firstRun)
    k = find(press>950);
    press(k) = press(k(1)-1);
    firstRun = false;
end

seaLevelhPA = 1013.25;
alt = 44330*(1-((press)/seaLevelhPA).^(0.1903));
alt = alt - alt(1);

% cut-off frequency
fc = 25;
fs = 100;
n = 10;

[b,a] = butter(n,fc/(fs/2));
altFiltered = filter(b,a,alt);

tmp = trapz(ay-9.81);
%speed = tmp(0:end-1).*(timeSec(2:end)-timeSec(1:end-1));
%plot (speed)
speed = (altFiltered(2:end)-altFiltered(1:end-1))./...
        (timeSec(2:end)-timeSec(1:end-1));

%% Plots of temp and pressure and altitude

displayInfos = true;

figure(2)
subplot(3,1,1)
plot(timeMillis,temp,'Linewidth',1.5)
ylabel('Temperature [�C]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
%ylim([-50 100]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
title('Thermometer and baromoter')
set(gca,'fontsize', 16);
grid on

subplot(3,1,2)
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

subplot(3,1,3)
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
ylabel('g_x [�/s]')
xlabel('Time [ms]');
xlim([minMillis 9300]);
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
ylabel('g_y [�/s]')
xlabel('Time [ms]');
xlim([minMillis 9300]);
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
ylabel('g_z [�/s]')
xlabel('Time [ms]');
xlim([minMillis 9300]);
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
ylabel('a_x [m/s^2]')
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
ylabel('a_y (Normal) [m/s^2]')
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
ylabel('a_z(Radial) [m/s^2]')
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
ylabel('a_y [m/s^2]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
ylim([-50 100]);
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
xlim([minMillis maxMillis]);
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
xlim([minMillis maxMillis]);
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
plot(timeMillis(1:end-1),speed,'Linewidth',1.5)
xlim([minMillis maxMillis]);
ylabel('speed [m/s]')
xlabel('Time [ms]');
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(tABcond1,'k','Brakes')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on