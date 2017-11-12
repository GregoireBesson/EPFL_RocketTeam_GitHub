function [z_hat] = measurment_model(obj, dt)
    % motion model based on motion equation
    % inputs:   obj, class containing previous sensors data
    %           dt, time increment between two update steps
    % output:   x_hat, state vector containing the estimated current states
    
    z_hat = zeros(3,1);
    
    z_hat(1) = 1; %%%% que mettre la dedans ?
    z_hat(2) = 1;
    z_hat(3) = 1;
    
end