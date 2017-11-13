clc
clear all
close all

load('compute_speed_data.mat');
load('log.mat');

timeSim = log(:,12)*1000;
accSim = log(:,13);

% time offset (motor starts burning at t=0ms)
tOffset = 1.820634e6;
t0 = 0;
% obtained graphically, coherent with expected burnout time
tBurnout = 1975;
% get apogee as the max of the altitude
[~, i] = max(alt);
tApogee = timeMillis(i);
% condition1 is t(a_y < 20g)+500ms  
tABcond1 = 2375;
% condition2 is activated 2200ms after the motor starts burning
tABcond2 = t0 + 2200;
tPara = 9198;

% keep only burn and flight
cut = timeMillis>t0&timeMillis<tPara;
ayG = ayG(cut);
alt = alt(cut);
timeMillis = timeMillis(cut);

%integrate the acceleration
speedacc = cumsum((ayG(1:end-1)-9.81).*(timeMillis(2:end)-timeMillis(1:end-1))/1000);
speedacc = [speedacc;0]; %0 avant ou apres?
%derive the altitude
%speedbaro = (alt(2:end)-alt(1:end-1))./((timeMillis(2:end)-timeMillis(1:end-1))/1000);
%speedbaro = [speedbaro;0]; %0 avant ou apres?

%integrate the speedacc (test) // c'est pas vraiement l'altitude, c'est la
%distance parcourue, 
altacc = cumsum((speedacc(1:end-1)).*(timeMillis(2:end)-timeMillis(1:end-1))/1000);
altacc = [altacc;0]; %0 avant ou apres?

hold on
grid on
%plot(timeMillis, alt,'Linewidth',1.5)
plot(timeMillis, ayG,'Linewidth',1.5)
plot(timeMillis, speedacc,'Linewidth',1.5)
plot(timeMillis, altacc,'Linewidth',1.5)
%plot(timeMillis, speedbaro,'Linewidth',1.5)
legend('Acc [m/s^2]', 'Velocity [m/s]', 'Altitude [m]')
xlabel('Time [ms]');
vline(t0,'r--','Burn')
vline(tBurnout,'r--','Burnout')
vline(tABcond1,'k','BrakesCond')
%vline(tABcond2,'k','BrakesCond2')
vline(tApogee,'b','Apogee')
vline(tPara,'g--','Para')
set(gca,'fontsize', 16);

figure
grid on
hold on
plot(timeMillis, ayG,'Linewidth',1.5)
plot(timeSim,accSim+9.81,'Linewidth',1.5)
set(gca,'fontsize', 16);
legend('Test Flight', 'Simulation')
xlabel('Time [ms]')
ylabel('Acceleration')
title('Simulated acceleration vs Recorded acceleration');

% get airbrakes start as the maximum of the drag during the flight
[~, i] = min(ayG);
tAB = timeMillis(i);

% keep only the flight phase with the airbrakes
cut2 = timeMillis>tAB&timeMillis<(tApogee); %j'ai enlev� 20 ms parce que le parachute fait un peu de la merde
ayG = ayG(cut2);
speedacc = speedacc(cut2);
figure
grid on
hold on
scatter(speedacc, ayG,'.');
p = polyfit(speedacc, ayG, 2);
x = min(speedacc):.01:max(speedacc);
f1 = polyval(p,x);
plot(x,f1,'Linewidth',3);
title('Drag acceleration vs speed with airbrakes');
ylabel('Drag acceleration [m/s^2]');
xlabel('Speed [m/s]');
set(gca,'fontsize', 16);
legend('data', '4th order polynom')
