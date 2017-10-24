function [ x ] = kalman_x( x, acc, alt, dt)

% x: actual state
% acc: measured acceleration
% alt: measured  alatitude
% dt: period

F = [1 dt dt^2; 0 1 dt; 0 0 0];
B = [0;0;1];

P = [.05 0; 0 .05];
H = [1 0 0];
R = 1;
S = 1;
z = alt;
acc_model = 0; % mettre la valeur du modèle
x(3) = acc_model;

x_hat = F * x;
y = z - H * x_hat;   %x_hat(2)
S = H*P*H' + R;
k = P * H' * inv(S);
x = x + k .* y;


end

