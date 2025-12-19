function [psi,theta,phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
% Check that R is a valid rotation matrix using IsRotationMatrix().

if ~IsRotationMatrix(R), return; end

theta = atan2(-R(3, 1),sqrt((R(1,1))^2+(R(2,1))^2));

if abs(abs(theta) - pi/2) < 1e-3
    warning('Representation is singular, gimbal lock, we will provide random angles');
    psi = rand*2*pi;
    phi = rand*2*pi; 

else
    psi = atan2(R(2,1), R(1,1));
    phi = atan2(R(3,2), R(3,3));
end
fprintf("YPR angles are: psi = %d , theta = %d , phi = %d \n",psi,theta,phi);
end

