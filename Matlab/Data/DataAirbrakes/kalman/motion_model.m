function [x_hat] = motion_model(obj, dt)
    % motion model based on motion equation
    % inputs:   obj, class containing previous states
    %           dt, time increment between two update steps
    % output:   x_hat, state vector containing the estimated current states
    
    x_hat = zeros(3,1);
    
    x_hat(1) = obj.x(1) + obj.x(2)*dt + obj.x(3)*dt^2;
    x_hat(2) = obj.x(2) + obj.x(3)*dt;
    x_hat(3) = obj.x(3);
    
end