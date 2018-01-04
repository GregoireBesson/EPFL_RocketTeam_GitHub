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

% timestamp read in microseconds
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

% log.mat is data logged from simulator, used for comparison
load('log.mat');

timeSim = log(:,12)*1000;
accSim = log(:,13);

%% Processing
% for the sake of comprehension, milliseconds will be used from now

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

% boundaries used for the plots 
minMillis = -1000;
maxMillis = 40000;
minSec = minMillis/1000;
maxSec = maxMillis/1000;

% removal of aberrant values on the pressure caused by parachute explosion
% on the first run 
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

% speed and altitude from integrating the acceleration:
% cut the vectors on the section that interest us the most: the flight
cut = (timeMillis>t0) & (timeMillis<tPara);
ayMS2Cut = ayMS2(cut);
altCut = alt(cut);
timeMillisCut = timeMillis(cut);
% integration is made using a cumulative sum of the acceleration times 
% delta_t which is not constant
velocityFromAcc = cumsum((ayMS2Cut(1:end-1)-9.81).*...
    (timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
% add a zero to keep same vector length
velocityFromAcc = [velocityFromAcc;0];
altFromAcc = cumsum((velocityFromAcc(1:end-1)).*...
    (timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
altFromAcc = [altFromAcc;0];

% determine the orientation of the rocket on the launchpad from acc
before_start = timeMillis>(t0-5.5e3) & timeMillis<(t0-0.5e3);
Pitch0 = atan2(mean(az(before_start)),-mean(ay(before_start)));
Yaw0 = atan2(mean(ax(before_start)),-mean(ay(before_start)));

% gyroscope calibration, need to enter begin and end times of a rest period
% (before or after the flight)
timeSpanBeforeStart = timeMillis(before_start);
t0_rest = timeSpanBeforeStart(1);
t1_rest = timeSpanBeforeStart(end);
[gx, gy, gz] = cal_gyr( gx, gy, gz, t0_rest, t1_rest, timeMillis);
gx = gx(cut);
gy = gy(cut);
gz = gz(cut);

% Compute angles from gyroscope integration
Pitch = Pitch0 + cumsum(gx(1:end-1).*...
    (timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
Pitch = [0; Pitch];
Roll = cumsum(gy(1:end-1).*...
    (timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
Roll = [0; Roll];
Yaw = Yaw0 + cumsum(gz(1:end-1).*...
    (timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
Yaw = [0; Yaw];

% compute the inclination of the rocket from the vertical
inclination = atan(sqrt(tan(Pitch).^2+tan(Yaw).^2));

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
altFromPitot = cumsum((velocityPitotCut(1:end-1)).*...
    (timeMillisCut(2:end)-timeMillisCut(1:end-1))/1000);
altFromPitot = [altFromPitot;0];

%% Plots of pressure and altitude

% boolean to display events on the plot
displayInfos = false;

figure(1)
title('Data from Baromoter')

subplot(2,1,1)
plot(timeSec,press,'Linewidth',1.5)
ylabel('Pressure [hPa]')
%xlabel('Time [ms]');
xlim([minSec maxSec]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(2,1,2)
hold on
plot(timeSec,alt,'Linewidth',1.5)
%plot(timeMillisCut(1:end-1),altFromPitot(1:end-1),'Linewidth',1.5)
%plot(timeMillisCut(1:end-1),altFromAcc(1:end-1),'Linewidth',1.5)
ylabel('Altitude [m]')
xlabel('Time [s]');
xlim([minSec maxSec]);
ylim([-10 260]);
%legend('From Baro','From Pitot','From Acc')
%if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout/1000,'r--','Burnout')
    vline(tApogee/1000,'b--','Apogee')
    vline(tPara/1000,'g--','Parachute')
%end
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
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
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
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(3,1,3)
plot(timeMillis,mz,'Linewidth',1.5)
ylabel('m_z [\muT]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

%% Plots of gyro on three axis

figure(4)
subplot(3,1,1)
plot(timeMillisCut,gx,'Linewidth',1.5)
ylabel('g_x [°/s]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
    vline(tPara,'g--','Para')
end
title('Angular rates')
set(gca,'fontsize', 16);
grid on

subplot(3,1,2)
plot(timeMillisCut,gy,'Linewidth',1.5)
ylabel('g_y [°/s]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(3,1,3)
plot(timeMillisCut,gz,'Linewidth',1.5)
ylabel('g_z [°/s]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1') 
    vline(tApogee,'c--','Apogee')
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
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
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
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
    vline(tPara,'g--','Para')
end
set(gca,'fontsize', 16);
grid on

subplot(3,1,3)
plot(timeMillis,azG,'Linewidth',1.5)
ylabel('a_z(Radial) [g]')
xlabel('Time [ms]');
xlim([minMillis maxMillis]);
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
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
if(displayInfos)
    vline(t0,'r--','Burn')
    vline(tBurnout,'r--','Burnout')
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
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
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
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
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
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
    vline(topen1,'k','Open1')
    vline(tclose1,'k','Close1')
    vline(tApogee,'c--','Apogee')
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
yyaxis left
plot(timeMillis,ayG,'Linewidth',1.5,'DisplayName','Acceleration')
ylabel('Acceleration [g]')
xlabel('Time [ms]');
xlim([minMillis tPara]);
set(gca,'fontsize', 16);
grid on
hold on
plot(timeMillis, AirbrakesSignal,'k', 'Linewidth',1.5,'DisplayName','Airbrakes Signal')
hold off

yyaxis right
plot(timeMillisCut(1:end-1),velocityFromAcc(1:end-1),'Linewidth',1.5,'DisplayName','Velocity');
ylabel('Velocity [m/s^2]')
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
load('logSimNoBrakes.mat')

SimAccNoBrakes = log(450:end,13);
SimSpeedNoBrakes = log(450:end,6);

figure(10)
close

% cut the time vector to find the period of braking
cut2 = (timeMillisCut>tBurnout+290) & (timeMillisCut<(tApogee));
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
scatter(velocityFromAcc(cutOpen),ayMS2Cut(cutOpen),'ro','Linewidth',2)
scatter(velocityFromAcc(cutClose),ayMS2Cut(cutClose),'bo','Linewidth',2)
x = min(velocityFromAcc):.01:max(velocityFromAcc);

% second order regression of the opened brakes datapoints
pOpen = polyfit(velocityFromAcc(cutOpen), ayMS2Cut(cutOpen), 2);
fOpen = polyval(pOpen,x);
plot(x,fOpen,'r--','Linewidth',1.5);
% second order regression of the closed brakes datapoints
pClose = polyfit(velocityFromAcc(cutClose), ayMS2Cut(cutClose), 2);
fClose = polyval(pClose,x);
plot(x,fClose,'b--','Linewidth',1.5);
pOpenNeuch = [-0.00184698786755789 0.00224022570886998 -0.491510493405732];
fOpenNeuch = polyval(pOpenNeuch,x);
%plot(x,fOpenNeuch,'Linewidth',2);
title('Drag acceleration vs velocity')
ylabel('Acceleration [m/s^2]')
xlabel('Veloctity [m/s]');
hold on
legend('open brakes', 'closed brakes', 'least squares open', 'least squares closed')
grid on
set(gca,'fontsize', 16);

% equation of motion of a free-flight object:
% x'' = (0.5*rho*Aref*Cd/m)*x'^2 = k*x'^2

% the first coefficient of the second order polynomial found by regression
% should be equal to the coeficient k.

% The Drag Coef. Cd can be therefore be computed:
% air density:
rho = 1.225;
%Aref = 0.0082;
% Rocket diameter (in meter):
D = 0.12;
Aref = pi*(D/2)^2;
% Dry mass of the rocket (in kg):
m = 2.5;

CdOpen = -pOpen(1)*m/(.5*rho*Aref);
CdClose = -pClose(1)*m/(.5*rho*Aref);

%% Superpose Acc / Speed / Alt

figure(11)
hold on
grid on
%plot(timeMillis, alt,'Linewidth',1.5)
plot(timeMillisCut, ayMS2Cut,'Linewidth',1.5)
plot(timeMillisCut, velocityFromAcc,'Linewidth',1.5)
plot(timeMillisCut, altFromAcc,'Linewidth',1.5)
%plot(timeMillis, speedbaro,'Linewidth',1.5)
legend('Acceleration [m/s^2]', 'Velocity [m/s]', 'Altitude [m]')
xlabel('Time [ms]');
xlim([t0 tApogee]);
ylim([-10 250]);
set(gca,'fontsize', 16);

%% Speed vs heigh

figure(12)
hold on 
grid on
plot(altCut(cut2),velocityFromAcc(cut2),'Linewidth',1.5)

%% Angles
RADtoDEG = 180/pi;
figure ()
hold on
grid on
plot(timeMillisCut,Pitch*RADtoDEG,'g','Linewidth',1.5)
%plot(timeMillisCut,Roll*RADtoDEG,'Linewidth',1.5)
plot(timeMillisCut,Yaw*RADtoDEG,'b','Linewidth',1.5)
plot(timeMillisCut,inclination*RADtoDEG,'r','Linewidth',1.5)
legend('Pitch', 'Yaw', 'Inclination');
xlabel('Time [ms]');
ylabel('Angles [Deg]')
set(gca,'fontsize', 16);
vline(7782,'r--','Apogee')    