function V = velocity_vector(q, q_dot, L1, L2)
% VELOCITY_VECTOR computes the velocity vector for a 2DOF RR manipulator
% using the physical parameters of the links

% Joint angles and velocities
q1 = q(1);
q2 = q(2);
q1_dot = q_dot(1);
q2_dot = q_dot(2);

% Physical parameters of the links
m1 = 1;
m2 = 0.8;
r1 = L1/2;
r2 = L2/2;

% Velocity vector
V1 = -m2*L1*r2*sin(q2)*q2_dot;
V2 = m2*L1*r2*sin(q2)*(q1_dot + q2_dot);
V = [V1;
     V2];
end