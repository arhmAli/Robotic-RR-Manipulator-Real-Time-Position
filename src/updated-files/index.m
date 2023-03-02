clc; clear; close all;

% Manipulator Parameters
L1 = 3; % Length of first link
L2 = 3; % Length of second link

% Trajectory Parameters
t = 0:0.01:10; % Time vector
x = cos(t); % X coordinate of the trajectory
y = sin(t); % Y coordinate of the trajectory
vx = -sin(t); % X velocity of the trajectory
vy = cos(t); % Y velocity of the trajectory
ax = -cos(t); % X acceleration of the trajectory
ay = -sin(t); % Y acceleration of the trajectory

% Simulation Parameters
dt = 0.01; % Time step
q0 = [0; pi/4]; % Initial joint angles
q_dot0 = [0; 0]; % Initial joint velocities
q_ddot0 = [0; 0]; % Initial joint accelerations
Kp = 20; % Proportional gain
Kd = 4; % Derivative gain

% Simulation
q = q0;
q_dot = q_dot0;
q_ddot = q_ddot0;
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
    plot([0, L1*cos(q(1)), L1*cos(q(1)) + L2*cos(q(1)+q(2))], [0, L1*sin(q(1)), L1*sin(q(1)) + L2*sin(q(1)+q(2))], 'LineWidth', 2);
    hold on;
    plot(x(1:i+1), y(1:i+1), 'r', 'LineWidth', 2);
    axis equal;
    xlim([-5, 7]);
    ylim([-5, 5]);
    xlabel('X');
    ylabel('Y');
    title('2DOF RR Manipulator Trajectory Following using Inverse Kinematics');
    hold off;
    pause(0.01);
end
