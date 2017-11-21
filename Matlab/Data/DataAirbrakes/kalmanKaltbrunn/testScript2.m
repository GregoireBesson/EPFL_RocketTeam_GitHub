% on dirait que Ca prend pas le 

% todo ajuster la masse de la rocket, (sans compter celle du moteur qui
% varie), 

%% Init

clc
clear all
close all

% resample everything at 100Hz (T=10ms)
load('dataKalman.mat')
t = (0:.01:timeMillis(end)/1000)';
altitude = interp1(timeMillis/1000, alt,t);
speed = interp1(timeMillis/1000, velocityPitot,t);
acceleration = interp1(timeMillis/1000, ayMS2,t);
load('thrust.mat')
thrust = interp1(thrust(:,1), thrust(:,2),t);
thrust(isnan(thrust))=0;

load('mass.mat')
mass = interp1(mass(:,1), mass(:,2),t);
mass(isnan(mass))=0.84; % todo trouver un moyen plus propre de faire ça
mass(1) = mass(2); % je sais pas pourquoi il y a 1.5 au début de la mass
% variation of the Cds over the time
load('tbrakes.mat')
CdOpen = 0.4069;
CdClose = 0.1992;
tbrakes = [0 topen1 topen1+150 tclose1 tclose1+150 topen2 topen2+150 tclose2 tclose2+150 topen3 topen3+150 tclose3 tclose3+150 topen4 topen4+150];
tbrakes = [tbrakes/1000 t(end)];
cdtmp = [CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen];
cd = interp1(tbrakes, cdtmp,t)
% initialisation du filtre de kalman
init = struct;
init.P = [.5 0 0; 0 .5 0; 0 0 .5];
init.H = eye(3);
init.x_hat = [0; 0; 0];
init.x = [0; 0; 0];
init.thrust = thrust; % value of the thrust in function of the time
init.mass_motor = mass; % values of the mass in function of the time
init.mass_rocket = 2.2; % mass of the rocket


kalman = kalman(init);

z = [altitude'; speed'; acceleration'];

%% Start plots

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
title('altitude');
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

%% Update Kalman

drawnow
for i = 1:length(z)-1
    
    % to be defined
    %R = [1000-i 0 0; 0 1000 0; 0 0 10];
    %Q = [1 0 0; 0 1 0; 0 0 1];
    R = [1000 0 0; 0 10 0; 0 0 5];   % baro hyper noisy
    Q = [1 0 0; 0 1 0; 0 0 50];     % prédicrion du thrust mauvaise
    [x_hat(:,i), x(:,i)] = update(kalman, z(:,i), R, Q, cd(i));
    
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

% Final plots
t = t(1:1:end-1);

subplot(3,1,1)
plot_pos = plot(t, x_hat(1,:)', t, z(1,1:end-1)', t, x(1,1:end-1)');
xlim([0 t(end)]);
%ylim([-20 260]);
title('altitude');
legend('Prediction','Measurment','Correction')
grid on

subplot(3,1,2)
plot_speed = plot(t, x_hat(2,:)',t, z(2,1:end-1)', t, x(2,1:end-1)');
legend('Prediction','Measurment','Correction')
xlim([0 t(end)]);
%ylim([-20 260]);
title('speed');
grid on

subplot(3,1,3)
plot_acc = plot(t, x_hat(3,:)',t, z(3,1:end-1)', t, x(3,1:end-1)');
legend('Prediction','Measurment','Correction')
xlim([0 t(end)]);
%ylim([-20 260]);
title('acceleration');
grid on

drawnow

