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

x_hat = zeros(3,length(t)-1);
x = zeros(3,length(t));

figure(2)
subplot(3,1,1)
plot_pos = plot(t, z(1,:));
xlim([0 t(end)]);
title('Altitude [m]');
grid on
subplot(3,1,2)
plot_speed = plot(t, z(2,:));
xlim([0 t(end)]);
title('Speed [m/s]');
grid on
subplot(3,1,3)
plot_acc = plot(t, z(3,:));
xlim([0 t(end)]);
title('Acceleration [m/s^2]');
xlabel('Time [s]')
grid on

%% Update Kalman

drawnow

t = t(1:end-1);

for i = 1:length(z)-1
    
    % to be defined
    R = [10 0 0; 0 5 0; 0 0 5];   % baro hyper noisy
    Q = [1 0 0; 0 1 0; 0 0 5];     % prédicrion du thrust mauvaise
    [x_hat(:,i), x(:,i)] = update(kalman, z(:,i), R, Q);
    
    subplot(3,1,1)
    plot_pos = plot(t(1:i), z(1,1:i)', t(1:i), x(1,1:i)');
    xlim([0 t(end)]);
    ylim([-20 260]);
    title('altitude');
    grid on

    subplot(3,1,2)
    plot_speed = plot(t(1:i), z(2,1:i)', t(1:i), x(2,1:i)');
    xlim([0 t(end)]);
    ylim([-20 260]);
    title('speed');
    grid on

    subplot(3,1,3)
    plot_acc = plot(t(1:i), z(3,1:i)', t(1:i), x(3,1:i)');
    xlim([0 t(end)]);
    ylim([-20 260]);
    title('acceleration');
    grid on

    drawnow
end

% Final plots

subplot(3,1,1)
plot_pos = plot(t, x_hat(1,:)','--', t, z(1,1:end-1)', t, x(1,1:end-1)');
xlim([0 t(end)]);
%ylim([-20 260]);
title('Altitude [m]');
legend('Prediction','Measurment','Estimator')
grid on
set(gca,'fontsize', 16);

subplot(3,1,2)
plot_speed = plot(t, x_hat(2,:)','--',t, z(2,1:end-1)', t, x(2,1:end-1)');
legend('Prediction','Measurment','Estimator')
xlim([0 t(end)]);
ylim([-20 75]);
title('Speed [m/s]');
grid on
set(gca,'fontsize', 16);

subplot(3,1,3)
plot_acc = plot(t, x_hat(3,:)','--',t, z(3,1:end-1)', t, x(3,1:end-1)');
legend('Prediction','Measurment','Estimator')
xlim([0 t(end)]);
ylim([-20 75]);
title('Acceleration [m/s^2]');
xlabel('Time [s]')
grid on
set(gca,'fontsize', 16);

drawnow

