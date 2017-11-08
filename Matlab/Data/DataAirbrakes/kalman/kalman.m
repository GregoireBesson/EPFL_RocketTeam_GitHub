
classdef kalman <handle
    properties
        P
        H
        B
        R
        Q
        S
        x
        Thrust = 0; % mettre les valeurs du thrust ici
        Mass = 0; % mettre les valeurs de la masse ici
    end
    methods
        function obj = kalman(init)
            obj.P = init.P;
            obj.H = init.H;
            obj.B = init.B;
            obj.R = init.R;
            obj.Q = init.Q;
            obj.S = init.S;
            obj.x = init.x;
        end

        function [ x ] = update(obj, z, dt)
            % z = [measured alt, measured speed, measured acc];
            % dt = period
            T = 1;
            m = 1;
            
            acc = (0.5*1.225*obj.x(2)^2*(0.0082*.3)+T)/m;
            % TODO ajouter aire brakes et CD brakes

            F = [1 dt acc*dt^2; 0 1 acc*dt; 0 0 acc];

            x_hat = F * obj.x;
            obj.P = F*obj.P*F' + obj.Q;

            y = z - obj.H * x_hat;   %x_hat(2)
            obj.S = obj.H*obj.P*obj.H' + obj.R;
            K = obj.P * obj.H' * inv(obj.S);
            obj.x = obj.x + K*y;
            obj.P = (eye(3)-K*obj.H)*obj.P
            x = obj.x;

        end
   end
end