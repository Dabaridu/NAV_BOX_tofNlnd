% KF3 Kalman Filter Example: Accelerometer + GPS (3D motion)
% Uses the KF3 class for state management

clear; clc;

dt = 0.1;
T = 20;
t = 0:dt:T;
N = numel(t);

%% True acceleration profile for each axis
aTrue_x = zeros(1,N);
aTrue_x(t >= 2 & t < 8) = 0.8;
aTrue_x(t >= 8 & t < 14) = -0.5;
aTrue_x(t >= 14 & t < 18) = 0.3;

aTrue_y = zeros(1,N);
aTrue_y(t >= 3 & t < 9) = 0.6;
aTrue_y(t >= 9 & t < 15) = -0.4;

aTrue_z = zeros(1,N);
aTrue_z(t >= 1 & t < 7) = 0.5;
aTrue_z(t >= 7 & t < 13) = -0.6;

% True 3D trajectory
xTrue = zeros(6,N); % [px, py, pz, vx, vy, vz]
for k = 2:N
	% Update velocities
	xTrue(4,k) = xTrue(4,k-1) + aTrue_x(k-1)*dt;
	xTrue(5,k) = xTrue(5,k-1) + aTrue_y(k-1)*dt;
	xTrue(6,k) = xTrue(6,k-1) + aTrue_z(k-1)*dt;
	% Update positions
	xTrue(1,k) = xTrue(1,k-1) + xTrue(4,k-1)*dt + 0.5*aTrue_x(k-1)*dt^2;
	xTrue(2,k) = xTrue(2,k-1) + xTrue(5,k-1)*dt + 0.5*aTrue_y(k-1)*dt^2;
	xTrue(3,k) = xTrue(3,k-1) + xTrue(6,k-1)*dt + 0.5*aTrue_z(k-1)*dt^2;
end

%% Sensor noise
sigmaAcc = 0.1;   % accelerometer std (m/s^2)
sigmaGPS = 1.0;   % GPS std (m)

% Measurements
accMeas_x = aTrue_x + sigmaAcc*randn(1,N);
accMeas_y = aTrue_y + sigmaAcc*randn(1,N);
accMeas_z = aTrue_z + sigmaAcc*randn(1,N);
gpsMeas = xTrue(1:3,:) + sigmaGPS*randn(3,N);

% Initialize KF3 filter for X axis
KF3_X = KF3;
KF3_X.Ts = dt;
KF3_X.x_hat = [0; 0; 0];
KF3_X.P = 1e3*eye(3);
KF3_X.F = [1 dt dt^2/2; 0 1 dt; 0 0 1];
sensorID_X = struct('H', [1 0 0], 'R', sigmaGPS^2);
KF3_X.Q = sigmaAcc^2 * [dt^4/4 dt^3/2 dt^2/2; dt^3/2 dt^2 dt; dt^2/2 dt 1];

% Initialize KF3 filter for Y axis
KF3_Y = KF3;
KF3_Y.Ts = dt;
KF3_Y.x_hat = [0; 0; 0];
KF3_Y.P = 1e3*eye(3);
KF3_Y.F = [1 dt dt^2/2; 0 1 dt; 0 0 1];
sensorID_Y = struct('H', [1 0 0], 'R', sigmaGPS^2);
KF3_Y.Q = sigmaAcc^2 * [dt^4/4 dt^3/2 dt^2/2; dt^3/2 dt^2 dt; dt^2/2 dt 1];

% Initialize KF3 filter for Z axis
KF3_Z = KF3;
KF3_Z.Ts = dt;
KF3_Z.x_hat = [0; 0; 0];
KF3_Z.P = 1e3*eye(3);
KF3_Z.F = [1 dt dt^2/2; 0 1 dt; 0 0 1];
sensorID_Z = struct('H', [1 0 0], 'R', sigmaGPS^2);
KF3_Z.Q = sigmaAcc^2 * [dt^4/4 dt^3/2 dt^2/2; dt^3/2 dt^2 dt; dt^2/2 dt 1];

xEst = zeros(6,N);
dt_array = ones(1,N) * dt;  % timestep for each measurement

% Add some variable timesteps: occasional delays
dt_array([50 100 150 200]) = 0.2;  % double timestep at these indices
dt_array([75 125 175]) = 0.05;     % half timestep at these indices

% Run filter for all three axes with variable timestep
for k = 1:N
	dt_k = dt_array(k);
	
	% Predict and update X axis
	KF3_X = kalman_update(KF3_X, dt_k, gpsMeas(1,k), sensorID_X, accMeas_x(k));
	xEst(1,k) = KF3_X.x_hat(1);
	xEst(4,k) = KF3_X.x_hat(2);
	
	% Predict and update Y axis
	KF3_Y = kalman_update(KF3_Y, dt_k, gpsMeas(2,k), sensorID_Y, accMeas_y(k));
	xEst(2,k) = KF3_Y.x_hat(1);
	xEst(5,k) = KF3_Y.x_hat(2);
	
	% Predict and update Z axis
	KF3_Z = kalman_update(KF3_Z, dt_k, gpsMeas(3,k), sensorID_Z, accMeas_z(k));
	xEst(3,k) = KF3_Z.x_hat(1);
	xEst(6,k) = KF3_Z.x_hat(2);
end

% Plot 3D trajectory
figure('Name','KF3 3D: Accelerometer + GPS');
subplot(2,2,1);
plot(t, xTrue(1,:), 'g', 'LineWidth', 1.5); hold on;
plot(t, gpsMeas(1,:), 'r.');
plot(t, xEst(1,:), 'b', 'LineWidth', 1.5);
grid on; ylabel('Position X (m)');
legend('True','GPS','KF3 Estimate','Location','best');

subplot(2,2,2);
plot(t, xTrue(2,:), 'g', 'LineWidth', 1.5); hold on;
plot(t, gpsMeas(2,:), 'r.');
plot(t, xEst(2,:), 'b', 'LineWidth', 1.5);
grid on; ylabel('Position Y (m)');
legend('True','GPS','KF3 Estimate','Location','best');

subplot(2,2,3);
plot(t, xTrue(3,:), 'g', 'LineWidth', 1.5); hold on;
plot(t, gpsMeas(3,:), 'r.');
plot(t, xEst(3,:), 'b', 'LineWidth', 1.5);
grid on; ylabel('Position Z (m)'); xlabel('Time (s)');
legend('True','GPS','KF3 Estimate','Location','best');

subplot(2,2,4);
plot3(xTrue(1,:), xTrue(2,:), xTrue(3,:), 'g-', 'LineWidth', 1.5); hold on;
plot3(gpsMeas(1,:), gpsMeas(2,:), gpsMeas(3,:), 'r.', 'MarkerSize', 4);
plot3(xEst(1,:), xEst(2,:), xEst(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('True','GPS','KF3 Estimate','Location','best');
view(45,45);

% Error metrics
pos_rmse_x = sqrt(mean((xEst(1,:) - xTrue(1,:)).^2));
pos_rmse_y = sqrt(mean((xEst(2,:) - xTrue(2,:)).^2));
pos_rmse_z = sqrt(mean((xEst(3,:) - xTrue(3,:)).^2));
vel_rmse_x = sqrt(mean((xEst(4,:) - xTrue(4,:)).^2));
vel_rmse_y = sqrt(mean((xEst(5,:) - xTrue(5,:)).^2));
vel_rmse_z = sqrt(mean((xEst(6,:) - xTrue(6,:)).^2));

fprintf('=== 3D Kalman Filter Results ===\n');
fprintf('Position RMSE - X: %.3f m, Y: %.3f m, Z: %.3f m\n', pos_rmse_x, pos_rmse_y, pos_rmse_z);
fprintf('Velocity RMSE - X: %.3f m/s, Y: %.3f m/s, Z: %.3f m/s\n', vel_rmse_x, vel_rmse_y, vel_rmse_z);
fprintf('Overall Position RMSE: %.3f m\n', sqrt(pos_rmse_x^2 + pos_rmse_y^2 + pos_rmse_z^2));

% Helper function to call kalman_update with variable timestep
function kf_out = kalman_update(kf_in, ts_var, z, sensorID, accel)
	% Update F and Q matrices dynamically based on variable timestep
	if abs(ts_var - kf_in.Ts) > 1e-6
		kf_in.F = [1 ts_var ts_var^2/2; 0 1 ts_var; 0 0 1];
		kf_in.Q = [ts_var^4/4 ts_var^3/2 ts_var^2/2; 
		           ts_var^3/2 ts_var^2 ts_var; 
		           ts_var^2/2 ts_var 1];
	end
	kf_in.Ts = ts_var;
	x_hat_last = kf_in.x_hat;
	
	% Prediction step
	x_pred = kf_in.F * x_hat_last;
	P_pred = kf_in.F * kf_in.P * kf_in.F' + kf_in.Q;
	
	% Update step
	H = sensorID.H;
	R = sensorID.R;
	y = z - H * x_pred;
	S = H * P_pred * H' + R;
	K = P_pred * H' / S;
	kf_in.x_hat = x_pred + K * y;
	kf_in.P = (eye(3) - K * H) * P_pred;
	
	kf_out = kf_in;
end