
classdef kalman <handle
    properties
        i
        P
        H
        R
        Q
        x_hat
        x
        mass
        thrust
    end
    methods
        function obj = kalman(init)
            obj.i = 0;
            obj.P = init.P;
            obj.H = init.H;
            obj.x_hat = init.x_hat;
            obj.x = init.x;
            obj.thrust = init.thrust;
            obj.mass = init.mass_motor + init.mass_rocket;
        end

        function [x_hat, x] = update(obj, z, R, Q)
            % z = [measured alt, measured speed, measured acc];
            % dt = period
            obj.i = obj.i+1;
            dt = .01;
            
            % TODO transmetre env.rho, roro.Aref et roro.Cd
            rho = 1.225;
            Aref = 0.0082;
            Cd = .4;
            dm = (obj.mass(obj.i+1)-obj.mass(obj.i))/dt;
            % TODO ajouter aire brakes et CD brakes
   
            % motion model that predicts the current state frome previous
            % // ajouter CD quand on aura fait une identification
            x_hat = motion_model(obj, dt);
            
            % Jacobian of the motion model
            % df x dot dot over d x dot
            dfxdd_dxd = (Cd * rho * Aref * obj.x(2)^2* + dm)/(obj.mass(obj.i));
            F = [1 dt-dfxdd_dxd*dt^2 dt^2; 0 dfxdd_dxd*dt dt; 0 dfxdd_dxd 1];
            
            obj.P = F*obj.P*F' + Q;

            y = z - obj.H * x_hat;  
            S = obj.H*obj.P*obj.H' + R;
            K = obj.P * (obj.H')/(S);
            obj.x = x_hat + K*y;
            obj.P = (eye(3)-K*obj.H)*obj.P;
            x = obj.x;

        end
   end
end