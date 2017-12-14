% on dirait que Ca prend pas le 

% todo ajuster la masse de la rocket, (sans compter celle du moteur qui
% varie), 

%% Init

clc
clear all
close all

% resample everything at 100Hz (T=10ms)
load('dataKalman.mat')
load('attitudeKalman.mat')

t = (0:.01:timeMillis(end)/1000)';
altitude = interp1(timeMillis/1000, alt,t);
speed = interp1(timeMillis/1000, velocityPitot,t);
acceleration = interp1(timeMillis/1000, ayMS2,t);
roll = interp1(timeMillis/1000, Roll,t);
pitch = interp1(timeMillis/1000, Pitch,t);
yaw = interp1(timeMillis/1000, Yaw,t);

% load expected thrust (given by the seller). data from Hassan's simulator
load('thrust.mat')
thrust = interp1(thrust(:,1), thrust(:,2),t);
thrust(isnan(thrust))=0;

% load the mass of the rocket over the time. data from Hassan's simulator
load('mass.mat')
mass = interp1(mass(:,1), mass(:,2),t);
mass(isnan(mass))=0.84; % todo trouver un moyen plus propre de faire �a
mass(1) = mass(2); % je sais pas pourquoi il y a 1.5 au d�but de la mass

% variation of the Cds over the time. load the time events of opening and
% closing from kaltbrunn test, to fit the theoritical model
load('tbrakes.mat')
% drag coefficients identified on the acc vs speed plot (least squares)
CdOpen = 0.7375;
CdClose = 0.3611;
% generate the Cd vector
tbrakes = [0 topen1 topen1+150 tclose1 tclose1+150 topen2 topen2+150 tclose2 tclose2+150 topen3 topen3+150 tclose3 tclose3+150 topen4 topen4+150];
tbrakes = [tbrakes/1000 t(end)];
cdtmp = [CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen];
cd = interp1(tbrakes, cdtmp,t)

rho = 1.225;
seaLevelhPA = 1013.25;


% initialisation du filtre de kalman
% structure from EKF Wikipedia
init = struct;
init.P = [.5 0 0; 0 .5 0; 0 0 .5];
init.H = eye(3);
init.x_hat = [0; 0; 0];
init.x = [0; 0; 0];
init.thrust = thrust; % value of the thrust in function of the time
init.mass_motor = mass; % values of the mass in function of the time
init.mass_rocket = 2.33; % mass of the rocket


kalman = kalman(init);
% angle between the rocket and the vertical axis
inclination = atan(sqrt(tan(pitch).^2+tan(yaw).^2));
% measurements projected on the vertical axis
%z = [altitude'; (speed.*cos(inclination))'; (acceleration.*cos(inclination))'-9.81];
z = [altitude'; speed'; acceleration'-9.81];

%% Start plots

figure
subplot(2,1,1)
plot(t, thrust);
title('expected thrust')
subplot(2,1,2)
plot(t, mass);
title('expected mass')
figure(2)
plot(inclination)
title('projection');

x = zeros(3,length(t));

figure(3)
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
% initialisation (not very important, is updated from the second timestep
R = [100 0 0; 0 50 0; 0 0 1];   % baro hyper noisy
Q = [1 0 0; 0 1 0; 0 0 10];     % pr�dicrion du thrust mauvaise

for i = 1:length(z)-1
    
    % update the EKF
    [x_hat(:,i), x(:,i)] = update(kalman, z(:,i), R, Q, cd(i), inclination(i));
    
    % compute the variance of the speed
    varspeed = 3.4630e+05 * 9/(400*rho*abs(x(2,i))); % abs to avoid boundary effect when speed =0
    
    % varalt = 0.0010 * 8436/(seaLevelhPA * ((1-x(1,i)/44330)^(5.255))^.8097); % prendre une constante pour le embeded, �a sert � rien d'�tre aussi pr�cis. cette ligne est utile juste pour justifier qu'on prend une constante
    varalt = 0.0742;
    % compute the covariance matrixes of the meausrement noise
    R = [varalt 0 0; 0 varspeed 0; 0 0 4.0707e-04];
    % compute the covariance matrixes of the process noise
    Q = [0.0001 0 0; 0 15 0; 0 0 .001];
    Q = [0.0001 0 0; 0 10 0; 0 0 10000];
%4.0707e-04

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

%% Final plots
t = t(1:1:end-1);

subplot(3,1,1)
plot_pos = plot(t, x_hat(1,:)', t, z(1,1:end-1)', t, x(1,1:end-1)');
yyaxis right
varalt = 0.0010 * 8436./(seaLevelhPA * ((1-x(1,1:end-1)/44330).^(5.255)).^.8097);
0.0010 * 8436./(seaLevelhPA * ((1-3000/44330).^(5.255)).^.8097);
plot(t, varalt)
ylim([0 1e-2])
xlim([0 t(end)]);
%ylim([-20 260]);
title('altitude');
legend('Prediction','Measurment','Correction', 'var')
grid on

subplot(3,1,2)
cosi = cos(inclination);
plot_speed = plot(t, x_hat(2,:)',t, z(2,1:end-1)', t, x(2,1:end-1)', t, x(2,1:end-1)'.*cosi(1:length(x(2,1:end-1))));
ylim([-10 70])
yyaxis right
varspeed = 3.4630e+05 * 9./(400*rho*abs(x(2,1:end-1)));
plot(t, varspeed)
ylim([-1e2 7e2])
legend('Prediction','Measurment','Estimator', 'vertical estimator', 'var')
xlim([0 t(end)]);
%ylim([-20 260]);
title('speed');
grid on

subplot(3,1,3)
plot_acc = plot(t, x_hat(3,:)',t, z(3,1:end-1)', t, x(3,1:end-1)');
yyaxis right
varacc = 4.2299e-06 * ones(1,length(t));
plot(t, varacc)
ylim([0 .00009])
legend('Prediction','Measurment','Correction', 'var')
xlim([0 t(end)]);
ylim([0 10e-6]);
title('acceleration');
grid on

drawnow

%% compute noise of the pitot

% pressure std at rest:   588.4761 [hPa]
% formula: v = (3/20)*sqrt(abs(2 * deltaP / RHO_AIR))
% derivative: dV/ddeltaP = 3*deltaP/(20*sqrt(2)*rho^2*abs(deltaP/rho)^(3/2)) % wolfram alpha
% expected deltaP for a given speed: deltaP = (v * (20/3))^2 *(rho/2)
% expected dV/ddeltaP for a given speed: = 3*(v * (20/3))^2 *(rho/2)/(20*sqrt(2)*rho^2*abs((v * (20/3))^2 *(rho/2)/rho)^(3/2))
% alternate form by wolfram alpha: dV/ddeltaP = 9/(400*rho*v)
% expected std of the speed from pitot: dV/ddeltaP * std(deltaP) =
% std(deltaP) * 9/(400*rho*v)

%3*deltaP/(20*sqrt(2)*rho^2*abs(deltaP/rho)^(3/2))

% acceleration std at rest:    0.0021 [m/s^2]

% barometer std at rest:     0.2725 [m](for launch pad)
% might be sufficient if we cinsider it as linear

% barometer std at rest:     0.0317 [hPa]
% formula: alt = 44330*(1-((press)/seaLevelhPA).^(0.1903));
% derivative dalt/dpress = 8436/(seaLevelhPA * (press/seaLevelhPA)^.8097)
% expected press for the altitude: press = seaLevelhPA*(1-alt/44330)^(1/.1903)
% expected dalt/dpress for a given alt: = 8436/(seaLevelhPA * ((seaLevelhPA*(1-alt/44330)^(1/.1903))/seaLevelhPA)^.8097)
% alternate form from wolfram: 8436/(seaLevelhPA * ((1-alt/44330)^(5.255))^.8097)
% expected std of the altitude: dalt/dpress * std(press) = 
% std(press) * 8436/(seaLevelhPA * ((1-alt/44330)^(5.255))^.8097)

