clc
clear all 
close all

g = 9.81;
% speed in m/s
v = 1:10:350;
% difference between
h = 1:10:2500;
%h = linspace(0,2500,length(v));
m = 17.6;
rho = 1.225;
Aref = pi*(0.12/2)^2;
factor = (2*m)/(rho*Aref);

for i=1:length(v) 
    for j=1:length(h)
        arg = (-2*g*h(j)*exp(-2*g*h(j)/(v(i)^2) ))/(v(i)^2);
        Cd(i,j) = -factor*(real(lambertw(-1, arg ))/(2*h(j)) - g/v(i)^2);
    end
end
   
%%
surf(h,v,Cd)
xlabel('h');
ylabel('v');
zlabel('Cd');
zlim([-2 10]);
caxis([-2 10]);


%% ODE45 

tspan = 0:0.005:30;

% initial conditions (x0=1000m, v0=300m/s)
state_0 = [1000; 300];

% Event function to stop at max height
%options = odeset('Events',@event_function);

[t, state]= ode45(@free_flight,tspan,state_0);

plot(t,state(:,1));

function state_dot = free_flight(t,state)
    % state = [x; x']
    % state_dot = [x'; x'']
    state_dot = zeros(2,1);
        
    % constants:
    rho = 1.225;
    Cd = 0.5;
    D = 0.12;
    Aref = pi*(D/2)^2;
    k = rho*Cd*Aref/2;
    m = 17.6;
    g = 9.81;
    
    % update state
    state_dot(1) = state(2);
    state_dot(2) = - (k/m)*state(2)^2 - g;
end