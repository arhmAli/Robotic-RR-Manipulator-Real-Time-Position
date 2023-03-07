//fixed bugs
clc; clear; close all;

% Manipulator Parameters
L1 = 3; % Length of first link
L2 = 3; % Length of second link

% Trajectory Parameters
t = 0:0.01:6.7; % Time vector
x = cos(t); % X coordinate of the trajectory
y = sin(t); % Y coordinate of the trajectory
vx = -sin(t); % X velocity of the trajectory
vy = cos(t); % Y velocity of the trajectory
ax = -cos(t); % X acceleration of the trajectory
ay = -sin(t); % Y acceleration of the trajectory

% Simulation Parameters
dt = t(2)-t(1); % Time step
q0 = [0; pi/4]; % Initial joint angles
q_dot0 = [0; 0]; % Initial joint velocities
q_ddot0 = [0; 0]; % Initial joint accelerations
Kp = 200; % Proportional gain
Kd = 40; % Derivative gain

% Simulation
q = q0;
q_dot = q_dot0;
q_ddot = q_ddot0;
figure;
hold on;
axis equal;
xlim([-15, 17]);
ylim([-15, 15]);
xlabel('X');
ylabel('Y');
title('2DOF RR Manipulator Trajectory Following using Inverse Kinematics');
for i = 1:length(t)-1
    % Desired end-effector position, velocity and acceleration
    p_d = [x(i+1); y(i+1)]
    p_dot_d = [vx(i+1); vy(i+1)]
    p_ddot_d = [ax(i+1); ay(i+1)]
    
    % Current end-effector position and Jacobian matrix
    [p, J] = forward_kinematics(q, L1, L2);
    
    
    % Current joint angle error
    e = p_d - p;
    e_dot = p_dot_d - J*q_dot;
    e_ddot = p_ddot_d - J*q_ddot;
    
    % Joint torques 
    M = mass_matrix(q, L1, L2);
    V = coriolis_matrix(q, q_dot, L1, L2);
    G = gravity_vector(q, L1, L2);
    tau = M*p_ddot_d + V*p_dot_d + G
    % Update joint accelerations, velocities and angles using inverse kinematics
    q_dot = inv(J)*([vx(i); vy(i)] - J*q_dot) + q_ddot*dt;
    q = q + q_dot*dt;
    
    % Plot the manipulator and trajectory
    x_base = 0;
y_base = 0;
    x_ee = L1*cos(q(1)) + L2*cos(q(1)+q(2));
    y_ee = L1*sin(q(1)) + L2*sin(q(1)+q(2));
plot([x_base, L1*cos(q(1)), x_ee], [y_base, L1*sin(q(1)), y_ee], 'LineWidth', 2);
plot(x(1:i+1), y(1:i+1), 'r', 'LineWidth', 2);
plot(x(i+1), y(i+1), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
plot(x_ee, y_ee, 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
legend({'Manipulator', 'Desired Trajectory', 'Desired End Effector', 'Actual End Effector'}, 'Location', 'northwest');
drawnow;
pause(0.01);
end

fprintf("The")
