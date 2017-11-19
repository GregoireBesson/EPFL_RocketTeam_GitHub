function [x_hat] = motion_model(obj, dt)
    % motion model based on motion equation
    % inputs:   obj, class containing previous states
    %           dt, time increment between two update steps
    % output:   x_hat, state vector containing the estimated current states
    
    x_hat = zeros(3,1);
    
    % expected acceleration
    dm = (obj.mass(obj.i+1)-obj.mass(obj.i))/dt;
    
    % TODO mettre un truc gloal pour otures ces valeurs
    rho = 1.225;
    Aref = 0.0082;
    Cd = .3;

    acc = (obj.thrust(obj.i) - Cd * .5 * rho * Aref * obj.x(2)^2* - dm/dt)/(obj.mass(obj.i));

%     x_hat(1) = obj.x(1) + obj.x(2)*dt + obj.x(3)*dt^2;
%     x_hat(2) = obj.x(2) + obj.x(3)*dt;
%     x_hat(3) = obj.x(3);
    x_hat(1) = obj.x(1) + obj.x(2)*dt + acc*dt^2;
    x_hat(2) = obj.x(2) + acc*dt;
    x_hat(3) = acc;
    
end