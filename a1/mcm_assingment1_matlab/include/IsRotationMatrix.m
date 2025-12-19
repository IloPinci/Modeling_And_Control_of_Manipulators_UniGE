function [isRotationMatrix] = IsRotationMatrix(R)
% The function checks that the input R is a valid rotation matrix, that is 
% a valid element of SO(3).
% Return true if R is a valid rotation matrix, false otherwise. In the
% latter case, print a warning pointing out the failed check.

isRotationMatrix = true;

R_t = R';
I = eye(3);
c = (norm((R_t * R) + (-I)) < 1e-3);

if ~c
    disp("The orthonormality condition is not met!");
    isRotationMatrix = false;
end

R_det = det(R);
c = (abs(R_det - 1) < 1e-3);

if ~c
    disp("The determinant condition is not met!");
    isRotationMatrix = false;
end
end