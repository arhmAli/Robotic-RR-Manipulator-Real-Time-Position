% Define the circle center and radius
circle_center = [2; 2];
circle_radius = 0.5;

theta_tf=2*pi;
theta_t_0=0;

%define the length of links
l_1=3;
l_2=3;

%Define the tf and the dt
tf=2;
dt=0.001;
t=0:dt:tf;
% Define the number of time steps for the simulation
num_steps = 300;
figure();

for i=1:1:num_steps
[x,y]=Circle_Traj(t(i),tf,theta_tf,theta_t_0);
theta = i * (2 * pi / num_steps);
end_effector_pos = circle_center + circle_radius * [cos(theta); sin(theta)];
% Calculate the joint angles using the inverse kinematics equations
[theta1,theta2]=inverseKinematic(end_effector_pos(1),end_effector_pos(2));
 theta_tf=theta1;
 theta_t_0=theta2;
 joint1_pos = [0; 0];
joint2_pos=dhparams(l_1,l_2,theta_tf,theta_t_0);
 plot([joint1_pos(1), joint2_pos(1), end_effector_pos(1)], ...
         [joint1_pos(2), joint2_pos(2), end_effector_pos(2)], '-o');
    title(sprintf('Time step: %d', i));
    axis([-3 3 -3 3])
    xlabel('x');
    ylabel('y');
    drawnow;
    pause(dt);
end
