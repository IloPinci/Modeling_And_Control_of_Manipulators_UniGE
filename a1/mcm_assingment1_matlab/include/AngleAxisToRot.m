function R = AngleAxisToRot(h,theta)
% The fuction implement the Rodrigues Formula
% Input: 
% h is the axis of rotation
% theta is the angle of rotation (rad)
% Output:
% R rotation matrix

%Make sure h is always a column vector
h = h(:);

%Make sure h has a norm of 1 and handle the 0 case
norm_h = sqrt(h(1)^2+h(2)^2+h(3)^2);
fprintf("Norm of vector h = %f \n",norm_h);
tol = 1e-3;
if norm_h < tol
    if theta < tol
        R = eye(3);
        disp("Theta is 0 so for all h we have no rotation, hence R is the identity");
        return
    else
    error("Axis is undefined since h is a 0 vector and theta is non zero");
    end
end
%Make sure we normalize h in case norm is not 1
h = h / norm_h;

%Implementing Rodrigues Formula
h_skew =    [0, -h(3), h(2); 
            h(3), 0, -h(1); 
            -h(2), h(1), 0];

anti_sym = h_skew*sin(theta);
sym = (1-cos(theta))*(h_skew*h_skew);
R = eye(3)+anti_sym+sym;
disp("The Rotation Matrix obtained is:");
disp(R);

end
