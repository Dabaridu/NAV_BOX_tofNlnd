classdef KF < matlab.System
    % KF Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object.

    % Public, tunable properties
    properties
        %model specific

        Ts = 1;

        x = [0 0 0]'; %innitialize state vector of [pos, vel, acc]
        F = [1 0 0; 0 1 0; 0 0 1]; %innitialize system dynamics matrix in state space
        Q = 1e-6*eye(3); %innitialize system covariance matrix of proccess noise
        P = 1e6*eye(3); %innitialize covariance matrix of state estimation error

        %measurement specific
        %sensorID = struct('H', H, 'R', R)


    end

    % Pre-computed constants or internal states
    properties (Access = private)
        
    end

    methods
        %choose method based on filter structure you want to use
        function kalman_step_predict_3(obj,Ts) 
            %Basic x,v,a estimation 
            %-----------For this specific filter dynamics---------
            obj.F = [1 Ts Ts^2/2; 
                     0  1  Ts;
                     0  0  1]

            obj.Q = [Ts^4/4 Ts^3/2 Ts^2/2; 
                     Ts^3/2 Ts^2   Ts;
                     Ts^2/2 Ts     1]

            %-----Standard KF code-------
        	% Prediction step
        	obj.x = obj.F * obj.x;
        	obj.P = obj.F * obj.P * obj.F' + obj.Q;
        end

        function kalman_step_update_3(obj, KFsens, z)
            %z            %measurement vector
            H = KFsens.H; %measurement transform matrics 
        	R = KFsens.R; %variance of measurement "sigma[e]^2"

            %-----Standard KF code-------
            %update step
        	y = z - H * obj.x;
        	S = H * obj.P * H' + R;
        	K = obj.P * H' / S;
        	obj.x = obj.x + K * y;
        	obj.P = (eye(3) - K * H) * obj.P;
        end

    end
end


