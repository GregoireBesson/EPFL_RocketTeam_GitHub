% clc
% clear all
% close all
% 
% % initialisation du filtre de kalman
% init = struct;
% init.P = [.5 0 0; 0 .5 0; 0 0 .5];
% init.H = [1 0 0 ; 0 1 0; 0 0 208];
% init.B = 0;
% init.R = [1e-6 0 0; 0 1e-6 0; 0 0 1e-6];
% init.Q = [1e-6 0 0; 0 1e-6 0; 0 0 1e-6];
% init.S = 0;
% init.x = [0; 0; 0];
% Thrust = 0; % mettre les valeurs du thrust ici
% Mass = 0; % mettre les valeurs de la masse ici
% 
% kalman = kalman(init);
% 
% x = zeros(3,10);
% for i = 1:500
%     % kalman.update(kalman, [acc speed alt], .01);
%     z = [i; 2*i; 3*i];
%     x(:,i)= update(kalman, z, .01);
% end
% 
% plot(x(1,:));
% hold on
% plot(x(2,:));
% plot(x(3,:));


clc
clear all
close all

% resample everything at 100Hz
load('dataKalman.mat')
t = (0:.01:timeMillis(end)/1000)';
altitude = interp1(timeMillis/1000, alt,t);
speed = interp1(timeMillis/1000, speedacc,t);
acceleration = interp1(timeMillis/1000, ayG,t);

load('motorData.mat')
motordata = [0 0; motordata]

Thrust = interp1(motordata(:,1), motordata(:,2),t);
Thrust(isnan(Thrust)) = 0;

load('motorMassData.mat')

Mass = interp1(t_mass, m,t);
Mass(1) = Mass(2); % parce qu'il y a un NaN au début
%%
clc
clear all
close all

load('100Hz_data.mat')

Mass(1) = Mass(2); % parce qu'il y a un NaN au début

% initialisation du filtre de kalman
init = struct;
init.P = [.5 0 0; 0 .5 0; 0 0 .5];
init.H = eye(3);
init.x = [0; 0; 0]
init.thrust = Thrust; % value of the thrust in function of the time
init.mass_motor = Mass; % values of the mass in function of the time
init.mass_rocket = 2; % mass of the rocket

kalman = kalman(init)

z = [altitude'; speed'; acceleration'];
z(:,1) = z(:,2); % because first values are NaN

% to be defined
R = eye(3)*.9;
Q = eye(3)*.1;

figure
subplot(2,1,1)
plot(t, Thrust);
title('expected thrust')
subplot(2,1,2)
plot(t, Mass);
title('expected mass')

x = zeros(3,length(t));

figure(2)
subplot(2,1,1)
plot_measures = plot(t, z');
xlim([0 t(end)]);
title('measurements');
grid on
subplot(2,1,2)
plot_kalman = plot(t, x');
title('filtered');
grid on
drawnow
for i = 1:length(z)
    % kalman.update(kalman, [acc speed alt], .01);
    x(:,i)= update(kalman, z(:,i), R, Q);
    
   subplot(2,1,1)
    plot_measures = plot(t(1:i), z(:,1:i)');
    xlim([0 t(end)]);
    ylim([-20 260]);
    grid on

   subplot(2,1,2)
    plot_kalman = plot(t(1:i), x(:,1:i)');
    xlim([0 t(end)]);
    ylim([-20 260]);
    grid on

   drawnow
end


