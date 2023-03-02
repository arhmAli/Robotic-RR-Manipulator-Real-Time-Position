function [p, J] = forward_kinematics(q, L1, L2)
% FORWARD_KINEMATICS computes the end-effector position and Jacobian matrix
% for a 2DOF RR manipulator

% Joint angles
q1 = q(1);
q2 = q(2);

% End-effector position
p = [L1*cos(q1) + L2*cos(q1+q2);
     L1*sin(q1) + L2*sin(q1+q2)];

% Jacobian matrix
J = [-L1*sin(q1)-L2*sin(q1+q2), -L2*sin(q1+q2);
     L1*cos(q1)+L2*cos(q1+q2), L2*cos(q1+q2)];
end