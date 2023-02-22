function [theta1,theta2] = inverseKinematic(x, y)
% INVERSEKINEMATICS calculates the joint angles of a 2-DoF planar RR
% manipulator given the end effector position and the lengths of the links.

% Calculate the joint angles using the inverse kinematics equations
L1=3;
L2=3;
r = sqrt(x^2 + y^2);
cos_theta2 = (r^2 - L1^2 - L2^2) / (2 * L1 * L2);
sin_theta2 = sqrt(1 - (cos_theta2)^2);
theta2 = atan2(sin_theta2, cos_theta2)
theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))
matrix=[cos_theta2 theta2;sin_theta2 theta1]
% Check if the joint angles are within their respective limits
d1=((theta1-cos_theta2)^2)+((theta2-sin_theta2)^2);
d2=((cos_theta2-theta1)^2)+((sin_theta2-theta2)^2);
if d1 < d2 
    endResult=d1
else
    endResult=d2
end
end
