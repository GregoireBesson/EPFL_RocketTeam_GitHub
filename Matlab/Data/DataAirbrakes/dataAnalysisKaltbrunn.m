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

load('log.mat');

timeSim = log(:,12)*1000;
accSim = log(:,13);

%% Processing

% time offset (motor starts burning at t=0ms) in ms
tOffset = 554906.830;
t0 = 0;

% events in ms
tm = 554972.368 - tOffset;
topen1 = 557485.792 - tOffset;
tclose1 = 558291.264 - tOffset;
topen2 = 559097.208 - tOffset;
tclose2 = 559907.908 - tOffset;
topen3 = 560721.528 - tOffset;
tclose3 = 561529.152 - tOffset;
topen4 = 562343.100 - tOffset;
tBurnout = 2090;
tApogee = 7634.7;
tPara = 10e3;

timeMicros = timeMicros - tOffset*1000;
timeMillis = timeMicros/1000;
timeSec = timeMillis/1000;
timeMin = timeSec/60;
minMillis = -1000;
maxMillis = 40000;
minSec = minMillis/1000;
maxSec = maxMillis/1000;

%removal of aberrant values on the first run
if (firstRun)
    press(10697)= press(10696);
    press(10698)= press(10696);
    press(10699)= press(10696);
end

% compute the altitude from the barometer
seaLevelhPA = 1013.25;
alt = 44330*(1-((press)/seaLevelhPA).^(0.1903));
alt = alt - alt(1);

% acceleration from LSBm/s^2 to m/s^2
accFactorUnits = 1/417.5;
axMS2 =  ax*accFactorUnits;
ayMS2 = -ay*accFactorUnits;
azMS2 =  az*accFactorUnits;
% convertion in g
axG = axMS2/9.81; 
ayG = ayMS2/9.81;
azG = azMS2/9.81;

% speed and altitude from integrating the acceleration
cut = timeMillis>t0&timeMillis<tPara;
ayMS2Cut = ayMS2(cut);
altCut = alt(cut);
timeMillisCut = timeMillis(cut);
velocityFromAcc = cumsum((ayMS2Cut(1:end-1)-9.81).*(timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
velocityFromAcc = [velocityFromAcc;0];
altFromAcc = cumsum((velocityFromAcc(1:end-1)).*(timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
altFromAcc = [altFromAcc;0];

%gyroscope calibraiton
%[gx, gy, gz] = cal_gyr(gx, gy, gz);

% compute the speed from the pitot sensor
deltaPoffset = deltaP(10);
deltaPmax = max(deltaP);
%deltaP = deltaP - deltaPoffset;
PRESSURE_SENSOR2_MAX = 103421;
PRESSURE_SENSOR2_MIN = 0;
deltaP = (deltaP - deltaPoffset)*...
                (PRESSURE_SENSOR2_MAX - PRESSURE_SENSOR2_MIN) / ...
                (deltaPmax - deltaPoffset) + PRESSURE_SENSOR2_MIN;
RHO_AIR = 1.225;
velocityPitot = (3/20)*sqrt(abs(2 * deltaP / RHO_AIR)); %%%%%%%%%%%%%%%%%%%%%% Why 3/20 ??

velocityPitotCut = velocityPitot(cut);
altFromPitot = cumsum((velocityPitotCut(1:end-1)).*(timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
altFromPitot = [altFromPitot;0];

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
hold on
plot(timeMillis,alt,'Linewidth',1.5)
plot(timeMillisCut,altFromPitot,'Linewidth',1.5)
plot(timeMillisCut,altFromAcc,'Linewidth',1.5)
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

%% Acceleration versus Airbrakes signal

AirbrakesSignal = ones(size(ay));
AirbrakesSignal = - AirbrakesSignal;
BrakeIndexes = find(timeMillis>=topen1 & timeMillis<tclose1 | ... 
                    timeMillis>=topen2 & timeMillis<tclose2 | ... 
                    timeMillis>=topen3 & timeMillis<tclose3 | ... 
                    timeMillis>=topen4 );
AirbrakesSignal(BrakeIndexes) = 1;

figure(7)
plot(timeMillis,ayG,'Linewidth',1.5,'DisplayName','Acceleration')
ylabel('Magnitude [g, -]')
xlabel('Time [ms]');
xlim([minMillis tPara]);
set(gca,'fontsize', 16);
grid on
hold on

plot(timeMillis, AirbrakesSignal, 'Linewidth',1.5,'DisplayName','Airbrakes Signal')
legend show

%% Speed from integration of acc VS speed from pitot sensor

figure(8)
title('Axial Velocity from integration VS Velocity from Pitot Tube');
plot(timeMillisCut,velocityFromAcc,'Linewidth',1.5,'DisplayName','From Acc');
hold on
plot(timeMillisCut,velocityPitot(cut),'Linewidth',1.5,'DisplayName','From Pitot');
xlim([minMillis tApogee]);
set(gca,'fontsize', 16);
grid on
legend show
ylabel('Veloctiy [m/s]')
xlabel('Time [ms]');

%% Alt from double integration of acc VS alt from barometer

figure(9)
title('Altitude from accelerometer VS alt. from barometer');
plot(timeMillisCut,altFromAcc,'--','Linewidth',2.5,'DisplayName','From Acc');
hold on
plot(timeMillisCut,alt(cut),'Linewidth',1.5,'DisplayName','From Baro');
xlim([t0 tApogee]);
set(gca,'fontsize', 16);
grid on
legend show
ylabel('Altitude [m]')
xlabel('Time [ms]');


%% Speed vs Drag Acceleration
figure(10)
close

cut2 = timeMillisCut>tBurnout+290&timeMillisCut<(tApogee);
% brakes need 150ms to change position
cutOpen = timeMillisCut>topen1+150&timeMillisCut<tclose1 ...
    | timeMillisCut>topen2+150&timeMillisCut<tclose2 ...
    | timeMillisCut>topen3+150&timeMillisCut<tclose3 ...
    | timeMillisCut>topen4+150&timeMillisCut<topen4+800;
cutClose = timeMillisCut>tBurnout+290&timeMillisCut<topen1 ...
    | timeMillisCut>tclose1+150&timeMillisCut<topen2 ...
    | timeMillisCut>tclose2+150&timeMillisCut<topen3 ...
    | timeMillisCut>tclose3+150&timeMillisCut<topen4;
figure(10)
hold on
grid on
scatter(velocityFromAcc(cutOpen),ayMS2Cut(cutOpen),'.')
scatter(velocityFromAcc(cutClose),ayMS2Cut(cutClose),'.')
x = min(velocityFromAcc):.01:max(velocityFromAcc);
pOpen = polyfit(velocityFromAcc(cutOpen), ayMS2Cut(cutOpen), 2);
fOpen = polyval(pOpen,x);
plot(x,fOpen,'Linewidth',2);
pClose = polyfit(velocityFromAcc(cutClose), ayMS2Cut(cutClose), 2);
fClose = polyval(pClose,x);
plot(x,fClose,'Linewidth',2);
pOpenNeuch = [-0.00184698786755789 0.00224022570886998 -0.491510493405732];
fOpenNeuch = polyval(pOpenNeuch,x);
plot(x,fOpenNeuch,'Linewidth',2);
title('Drag Acceleration vs speed')
ylabel('Acceleration [m/s^2]')
xlabel('Speed [m/s]');
legend('open brakes', 'closed brakes', 'least squares open', 'least squares closed', 'least squares small brakes open')
grid on
set(gca,'fontsize', 16);

% mettre le calcul complet, je sais pas si on change Aref
    rho = 1.225;
    Aref = 0.0082;

CdOpen = -pOpen(1)/(.5*rho*Aref)
CdClose = -pClose(1)/(.5*rho*Aref)

%% Superpose Acc / Speed / Alt

figure(11)
hold on
grid on
%plot(timeMillis, alt,'Linewidth',1.5)
plot(timeMillisCut, ayMS2Cut,'Linewidth',1.5)
plot(timeMillisCut, velocityFromAcc,'Linewidth',1.5)
plot(timeMillisCut, altFromAcc,'Linewidth',1.5)
%plot(timeMillis, speedbaro,'Linewidth',1.5)
legend('Acc [m/s^2]', 'Velocity [m/s]', 'Altitude [m]')
xlabel('Time [ms]');
xlim([t0 tApogee]);
set(gca,'fontsize', 16);