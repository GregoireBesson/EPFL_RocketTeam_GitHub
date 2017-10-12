% Data Analysis of a rocket test flight
% Sensors: 6DoF IMU, Magnetometer, Barometer
% Author: Grégoire Besson
% Date: 09/10/2017

clc
clear all
close all

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
press = M(:,12);

%% Processing

timeSec = timeMillis/1000;
timeMin = timeSec/60;
minSec = 1810;
maxSec = 1870;
tStartSec = 1820;
ayG = -ay/(208.77) ;

%removal of aberrant values
%k = find(press>9.5e4);
%press(k) = press(k(1)-1);

seaLevelhPA = 1013.25;
alt = 44330*(1-((press/100)/seaLevelhPA).^(0.1903));
alt = alt - alt(1);

% cut-off frequency
fc = 25;
fs = 100;
n = 10;

[b,a] = butter(n,fc/(fs/2));
altFiltered = filter(b,a,alt);

speed = (altFiltered(2:end)-altFiltered(1:end-1))./(timeSec(2:end)-timeSec(1:end-1));

%% Plots

subplot(4,1,1)
%plot(timeSec,temp,'DisplayName','Temperature [°C]')
plot(timeSec,ayG,'Linewidth',1.5)
ylabel('Acceleration [g]')
xlim([minSec maxSec]);
set(gca,'fontsize', 16);

subplot(4,1,2)
plot(timeSec,press,'Linewidth',1.5)
xlim([minSec maxSec]);
ylabel('Pressure [PA]')
xlabel('Time [sec]');
set(gca,'fontsize', 16);

subplot(4,1,3)
plot(timeSec,altFiltered,'Linewidth',1.5)
xlim([minSec maxSec]);
ylabel('Alttitude [m]')
xlabel('Time [sec]');
set(gca,'fontsize', 16);

subplot(4,1,4)
plot(timeSec(1:end-1),speed,'Linewidth',1.5)
xlim([minSec maxSec]);
ylabel('speed [m/s]')
xlabel('Time [sec]');
set(gca,'fontsize', 16);