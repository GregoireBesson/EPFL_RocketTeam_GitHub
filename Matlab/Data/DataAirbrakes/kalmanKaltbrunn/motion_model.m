function [x_hat] = motion_model(obj, Cd, dt, inclination)
    % motion model based on motion equation
    % inputs:   obj, class containing previous states
    %           dt, time increment between two update steps
    % output:   x_hat, state vector containing the estimated current states
    
    x_hat = zeros(3,1);
    
    % expected acceleration
    dm_dt = (obj.mass(obj.i+1)-obj.mass(obj.i))/dt;
    
    % TODO mettre un truc gloal pour otures ces valeurs
    rho = 1.225;
    Aref = 0.0082;
    Aref = 0.0113;
    %Aref = 0.03;

    % ici j'ai changé dm/dt en dm*obj.x(2) car le dt on le fait deja en haut
    % et la dérivée de la masse est multipliée par la vitesse

    % acc = (obj.thrust(obj.i) - Cd * .5 * rho * Aref * obj.x(2)^2 - dm_dt*obj.x(2))/(obj.mass(obj.i));
    acc = (obj.thrust(obj.i) - Cd*.5 * rho * Aref * obj.x(2)^2)/(obj.mass(obj.i))-9.81;

%     x_hat(1) = obj.x(1) + obj.x(2)*dt + obj.x(3)*dt^2;
%     x_hat(2) = obj.x(2) + obj.x(3)*dt;
%     x_hat(3) = obj.x(3);
    x_hat(1) = obj.x(1) + obj.x(2)*cos(inclination)*dt + acc*cos(inclination)*dt^2;
    x_hat(2) = obj.x(2) + acc*dt;
    x_hat(3) = acc;
    
end