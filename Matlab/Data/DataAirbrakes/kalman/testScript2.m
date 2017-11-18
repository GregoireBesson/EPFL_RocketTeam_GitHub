% on dirait que Ca prend pas le 

% todo ajuster la masse de la rocket, (sans compter celle du moteur qui
% varie), 

clc
clear all
close all

% resample everything at 100Hz
load('dataKalman.mat')
t = (0:.01:timeMillis(end)/1000)';
altitude = interp1(timeMillis/1000, alt,t);
speed = interp1(timeMillis/1000, speedacc,t);
acceleration = interp1(timeMillis/1000, ayG,t);
load('thrust.mat')
thrust = interp1(thrust(:,1), thrust(:,2),t);
thrust(isnan(thrust))=0;

load('mass.mat')
mass = interp1(mass(:,1), mass(:,2),t);
mass(isnan(mass))=0.84; % todo trouver un moyen plus propre de faire ça
mass(1) = mass(2); % je sais pas pourquoi il y a 1.5 au début de la mass
% initialisation du filtre de kalman
init = struct;
init.P = [.5 0 0; 0 .5 0; 0 0 .5];
init.H = eye(3);
init.x = [0; 0; 0]
init.thrust = thrust; % value of the thrust in function of the time
init.mass_motor = mass; % values of the mass in function of the time
init.mass_rocket = 2; % mass of the rocket

kalman = kalman(init)

z = [altitude'; speed'; acceleration'];


figure
subplot(2,1,1)
plot(t, thrust);
title('expected thrust')
subplot(2,1,2)
plot(t, mass);
title('expected mass')

x = zeros(3,length(t));

figure(2)
subplot(3,1,1)
plot_pos = plot(t, z(1,:));
xlim([0 t(end)]);
title('speed');
grid on
subplot(3,1,2)
plot_speed = plot(t, z(2,:));
xlim([0 t(end)]);
title('speed');
grid on
subplot(3,1,3)
plot_acc = plot(t, z(3,:));
xlim([0 t(end)]);
title('acceleration');
grid on

drawnow
for i = 1:length(z)
    
    % to be defined
    R = [1000-i 0 0; 0 1000 0; 0 0 10];
    Q = [1 0 0; 0 1 0; 0 0 1];

    x(:,i)= update(kalman, z(:,i), R, Q);
    
%     subplot(3,1,1)
%     plot_pos = plot(t(1:i), z(1,1:i)', t(1:i), x(1,1:i)');
%     xlim([0 t(end)]);
%     ylim([-20 260]);
%     title('altitude');
%     grid on
% 
%     subplot(3,1,2)
%     plot_speed = plot(t(1:i), z(2,1:i)', t(1:i), x(2,1:i)');
%     xlim([0 t(end)]);
%     ylim([-20 260]);
%     title('speed');
%     grid on
% 
%     subplot(3,1,3)
%     plot_acc = plot(t(1:i), z(3,1:i)', t(1:i), x(3,1:i)');
%     xlim([0 t(end)]);
%     ylim([-20 260]);
%     title('acceleration');
%     grid on
% 
%     drawnow
end
subplot(3,1,1)
plot_pos = plot(t, z(1,:)', t, x(1,:)', t, cumsum(z(2,:))*.01);
xlim([0 t(end)]);
ylim([-20 260]);
title('altitude');
grid on

subplot(3,1,2)
plot_speed = plot(t, z(2,:)', t, x(2,:)');
xlim([0 t(end)]);
ylim([-20 260]);
title('speed');
grid on

subplot(3,1,3)
plot_acc = plot(t, z(3,:)', t, x(3,:)');
xlim([0 t(end)]);
ylim([-20 260]);
title('acceleration');
grid on

drawnow

