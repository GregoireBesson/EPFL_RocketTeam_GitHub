%% Main 1
clear all; clc; close all

% Global varialbes
global env
global log

% Create rocket class
roro = rocket(init_rocket());% creates class with the initial values
%% Initialize motor
motor_init( roro ); %loads rocket motor

%%  Initialize Environmental variables 
% optional argument: Elevation(m) Temperature(C)and Pressure(Pa)
env = environement(350, 15, 96000, roro );

%% Phase: Accent
load('airbrakes_controller_table.mat')
controller = struct;
controller.table = table;
controller.v = v;
controller.Cd = Cd;
tend = 22;  %simulate 30 seconds
[t, state] = accent_calc(roro,tend,controller);

%% Plot the trajectory
figure(1);
plot(t,state(:,3))
xlabel('Time(s)')
ylabel('Height (m)')
 
figure(2);
plot3(state(:,1),state(:,2),state(:,3))
xlabel('x(m)')
ylabel('y (m)')
zlabel('Height (m)')
axis([-1500 1500 -1500 1500 0 3000])
h_max=max(state(:,3))


%%
extract_data ( state,t);
time = log(:,12);
acceleration = log(:,13);
speed = log(:,6);

%% Plot the speed
figure
hold on
plot(time,speed)
xlabel('Time(s)')
ylabel('Speed [m/s]')
legend('Simulated speed')
grid on

%%  Plot the acceleration
% figure(3)
% 
% plot(log(:,10),log(:,2))
% 
% xlabel('Time')
% ylabel('Value1, Value2')
% %axis([0 20 0.0 1])
% % 
figure
hold on
plot(time,acceleration)
xlabel('Time(s)')
ylabel('Acceleration [m/s^2]')
% yyaxis right
% plot(time,Cd_log)
% ylabel('airbrakes drag coefficient')
legend('Simulation acceleration')
grid on
%t2 = log(:,12);
% roll_acel2 = log(:,13);
% indStart=find(t2>4,1)
% indEnd=find(t2>7,1)
% plot(t2(indStart:indEnd),roll_acel2(indStart:indEnd))
% hold on

%% Plot flight and stability data
plotData(log, roro);

%% Save data for Cd Table
% h07 = h_max - state(1:end-1,3);
% s07 = state(:,10)/roro.Mass; % P = mv -> v = P/m
%% Plot the lookup table values
figure
hold on
grid on
scatter(h_max - state(1:end,3),state(:,10)/roro.Mass,'.');
scatter(controller.table(:,1),controller.v,'.');
scatter(controller.table(:,2),controller.v,'.');
scatter(controller.table(:,3),controller.v,'.');
scatter(controller.table(:,4),controller.v,'.');
scatter(controller.table(:,5),controller.v,'.');
scatter(controller.table(:,6),controller.v,'.');
scatter(controller.table(:,7),controller.v,'.');
scatter(controller.table(:,8),controller.v,'.');
legend('Simulator','Cd = 0','Cd = 0.1','Cd = 0.2','Cd = 0.3',...
                   'Cd = 0.4','Cd = 0.5','Cd = 0.6','Cd = 0.7') 
xlabel('Distance to apogee [m]')
ylabel('Veloctiy [m/s]')
set(gca,'fontsize', 16);
