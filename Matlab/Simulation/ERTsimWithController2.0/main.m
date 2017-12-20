%% Main 2.0

% Rocket simulator created for IREC 
% by Hassan Arif (EPFL)with contributioins form Moritz Zimmermann (ETHZ)
clear all; clc; close all

% Global varialbes


global env
global log
% Create rocket class
roro = rocket(init_rocket());% creates class with the initial values
% Loads rocket motor
motor_init( roro ); 
% Initilize Environmental variables 
% optional argument: Elevation(m) Temperature(C)and Pressure(Pa)
env = environement(1400, 42, 86200, roro );  % (350, 15, 99490, roro );

%% Phase: Accent
load('airbrakes_controller_table.mat')
controller = struct;
controller.table = table;
controller.v = v;
controller.Cd = Cd;

tend=30;
[t, state] = accent_calc(roro,tend,controller);
%%
% figure(1);
plot(t,state(:,3))
xlabel('Time(s)')
ylabel('Height (m)')
% trejectory
figure(2);
plot3(state(:,1),state(:,2),state(:,3))
xlabel('x(m)')
ylabel('y (m)')
zlabel('Height (m)')
axis([-500 500 -500 500 0 3200])
h_max=max(state(:,3))


%%
extract_data ( state,t);

%% Debugging plots
% figure(3)
% 
% plot(log(:,10),log(:,2))
% 
% xlabel('Time')
% ylabel('Value1, Value2')
% %axis([0 20 0.0 1])
% % 
% figure(4)
% plot(log(:,10),log(:,1))
% xlabel('Time')
% ylabel('Value1, Value2')

%% Plot flight and stability data
plotData(log, roro);

%%
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
legend('simulator','Cd = 0','Cd = 0.1','Cd = 0.2','Cd = 0.3',...
                   'Cd = 0.4','Cd = 0.5','Cd = 0.6','Cd = 0.7') 
xlabel('distance to apogee [m]')
ylabel('speed [m/s]')

