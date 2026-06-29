% KF3 example problem: estimate 1D motion (position, velocity, acceleration)
% from noisy position measurements and plot the result.

clear; clc; close all;

dt = 0.1;
T = 20;
t = 0:dt:T;

% True motion with constant acceleration
a_true = 0.8;
x_true = 2 + 1.5*t + 0.5*a_true*t.^2;

% Noisy measurements
sigma_meas = 2.0;
z = x_true + sigma_meas*randn(size(x_true));

% KF3 state: [position; velocity; acceleration]
F = [1 dt 0.5*dt^2;
     0 1  dt;
     0 0  1];
H = [1 0 0];
Q = diag([1e-4 1e-3 1e-2]);
R = sigma_meas^2;

n = numel(t);
x_hat = zeros(3, n);
P = eye(3) * 10;
x_hat(:,1) = [z(1); 0; 0];

for k = 2:n
    x_pred = F * x_hat(:,k-1);
    P_pred = F * P * F' + Q;

    y = z(k) - H * x_pred;
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;

    x_hat(:,k) = x_pred + K * y;
    P = (eye(3) - K * H) * P_pred;
end

figure();
plot(t, x_true, 'b-', 'LineWidth', 2); hold on;
plot(t, z, 'r.', 'MarkerSize', 10);
plot(t, x_hat(1,:), 'g-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Position');
title('KF3 Example: Tracking Noisy Position Measurements');
legend('True position', 'Noisy measurements', 'KF3 estimate', 'Location', 'northwest');

figure();
subplot(2,1,1);
plot(t, x_hat(2,:), 'm-', 'LineWidth', 2);
grid on;
ylabel('Velocity');
title('KF3 Estimated Velocity and Acceleration');

subplot(2,1,2);
plot(t, x_hat(3,:), 'k-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Acceleration');