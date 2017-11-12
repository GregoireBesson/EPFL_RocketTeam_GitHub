clc
clear all
close all

% initialisation du filtre de kalman
init = struct;
init.P = [.5 0 0; 0 .5 0; 0 0 .5];
init.H = [1 0 0 ; 0 1 0; 0 0 1];
init.B = 0;
init.R = [1e-6 0 0; 0 1e-6 0; 0 0 1e-6];
init.Q = [1e-6 0 0; 0 1e-6 0; 0 0 1e-6];
init.S = 0;
init.x = [0; 0; 0];
Thrust = 0; % mettre les valeurs du thrust ici
Mass = 0; % mettre les valeurs de la masse ici

kalman = kalman(init);

x = zeros(3,10);
for i = 1:10
    % kalman.update(kalman, [acc speed alt], .01);
    z = [i; 2*i; 3*i];
    x(:,i)= update(kalman, z, .01);
end



