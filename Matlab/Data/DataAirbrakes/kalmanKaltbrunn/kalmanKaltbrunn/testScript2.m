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
mass(isnan(mass))=0.84; % todo trouver un moyen plus propre de faire �a
mass(1) = mass(2); % je sais pas pourquoi il y a 1.5 au d�but de la mass
% variation of the Cds over the time
load('tbrakes.mat')
CdOpen = 0.4069;
CdClose = 0.1992;
tbrakes = [0 topen1 topen1+150 tclose1 tclose1+150 topen2 topen2+150 tclose2 tclose2+150 topen3 topen3+150 tclose3 tclose3+150 topen4 topen4+150];
tbrakes = [tbrakes/1000 t(end)];
cdtmp = [CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen CdClose CdClose CdOpen CdOpen];
cd = interp1(tbrakes, cdtmp,t)

rho = 1.225;
seaLevelhPA = 1013.25;


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
% initialisation
R = [100 0 0; 0 50 0; 0 0 1];   % baro hyper noisy
Q = [1 0 0; 0 1 0; 0 0 10];     % pr�dicrion du thrust mauvaise

for i = 1:length(z)-1
    
    % to be defined
    [x_hat(:,i), x(:,i)] = update(kalman, z(:,i), R, Q, cd(i));
    
    varspeed = 3.4630e+05 * 9/(400*rho*x(2,i));
    varalt = 0.0010 * 8436/(seaLevelhPA * ((1-x(1,i)/44330)^(5.255))^.8097); % prendre une constante pour le embeded, �a sert � rien d'�tre aussi pr�cis. cette ligne est utile juste pour justifier qu'on prend une constante
    varalt = 0.0742;
    % je les mets apr�s sinon j'ai pas de valeurs au premier tour
    R = [varalt 0 0; 0 varspeed 0; 0 0 4.0707e-04];   % baro hyper noisy
    Q = [0.001 0 0; 0 15 0; 0 0 4.e-04];     % pr�dicrion du thrust mauvaise
    %Q = [.000001 0 0; 0 10000 0; 0 0 1000];     % pr�dicrion du thrust mauvaise
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

% Final plots
t = t(1:1:end-1);

subplot(3,1,1)
plot_pos = plot(t, x_hat(1,:)', t, z(1,1:end-1)', t, x(1,1:end-1)');
yyaxis right
varalt = 0.0010 * 8436./(seaLevelhPA * ((1-x(1,1:end-1)/44330).^(5.255)).^.8097);
plot(t, varalt)
ylim([0 1e-2])
xlim([0 t(end)]);
%ylim([-20 260]);
title('altitude');
legend('Prediction','Measurment','Correction', 'std')
grid on

subplot(3,1,2)
plot_speed = plot(t, x_hat(2,:)',t, z(2,1:end-1)', t, x(2,1:end-1)');
yyaxis right
varspeed = 3.4630e+05 * 9./(400*rho*x(2,1:end-1));
plot(t, varspeed)
ylim([0 1e3])
legend('Prediction','Measurment','Correction', 'std')
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
legend('Prediction','Measurment','Correction', 'std')
xlim([0 t(end)]);
%ylim([-20 260]);
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

