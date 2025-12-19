function [h,theta] = RotToAngleAxis(R)
% Given a rotation matrix this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis)
% Check that R is a valid rotation matrix using IsRotationMatrix()

%Default values in case isRotationMatrix is false
h = [NaN; NaN; NaN];
theta = NaN;

if ~IsRotationMatrix(R)
    disp("This is not a valid Rotation Matrix. Provide a new one!");
    error("This is not a valid Rotation Matrix. Provide a new one!");
end

%Computing theta
theta = acos((trace(R)-1)/2); % included in [0,pi]
if theta<1e-3
    disp("h is arbitrary we can have infinite possibilities");
    h = [1;0;0];
   
elseif abs(theta-pi)<1e-3
    I = eye(3);
    hh_t = (R+I)/2;

    hx2 = hh_t(1, 1);
    hy2 = hh_t(2, 2);
    hz2 = hh_t(3, 3);

    hx = sqrt(hx2);
    hy = sqrt(hy2);
    hz = sqrt(hz2);

    %Determine the sign
    if abs(hx) > 1e-5
        if (hx * hy) * sign(R(1, 2)) < 0
            hy = -hy;
        end
        if (hx * hz) * sign(R(1, 2)) < 0
            hz = -hz;
        end
    elseif abs(hy) > 1e-5
        if (hx * hz) * sign(R(1, 2)) < 0
            hz = -hz;
        end
    end

    h = [hx; hy; hz];

else
    h = vex((R-R')/(2*sin(theta)));
    h = h/norm(h);
end
disp("The Angle Axis using RotToAngleAxis is:")
disp(h);
disp(theta);
end


 
function a = vex(S_a)
% input: skew matrix S_a (3x3)
% output: the original a vector (3x1)
a = [S_a(3,2);S_a(1,3);S_a(2,1)];
end