function [R] = YPRToRot(psi, theta, phi)
% The function compute the rotation matrix using the YPR (yaw-pitch-roll)
% convention, given psi, theta, phi.
% Input:
% psi angle around z axis (yaw)
% theta angle around y axis (theta)
% phi angle around x axis (phi)
% Output:
% R rotation matrix
Rz =[cos(psi)  -sin(psi)  0;
     sin(psi)  cos(psi)  0;
     0  0  1];

Ry = [cos(theta)  0  sin(theta);
      0  1  0;
      -sin(theta)  0  cos(theta)];

Rx = [1  0  0;
      0  cos(phi)  -sin(phi);
      0  sin(phi)  cos(phi)];

R = Rz*Ry*Rx;
disp("Rotation matrix from YPR:");
disp(R);
end