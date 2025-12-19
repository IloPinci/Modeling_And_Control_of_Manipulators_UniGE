addpath('include');
%% 1.1 Angle-axis to rot
clc;
clear;
% 1.2
disp("Question 1.2:");
R12 = AngleAxisToRot([1;0;0],pi/2);

% 1.3
disp(" ");
disp("Question 1.3:");
R13 = AngleAxisToRot([0;0;1],pi/3);

%1.4
disp(" ");
disp("Question 1.4:");
rho = [-pi/3;-pi/6;pi/3];
theta = sqrt(rho(1)^2+rho(2)^2+rho(3)^2)
if theta>1e-3 
    h = rho/theta 
else
    disp("Theta is zero, h is arbitary.")
    h=[1;0;0];
end
R14 = AngleAxisToRot(h,theta);

%% 1.2 Rot to angle-axis
clc;
clear;
% 2.2
disp(" ");
disp("Question 2.2:");
R22 = [1  0  0;
       0  0 -1;
       0  1  0];
V22 = RotToAngleAxis(R22);

% 2.3
disp(" ");
disp("Question 2.3:");
R23 = [0.5   -sqrt(3)/2   0;
       sqrt(3)/2  0.5     0;
       0      0          1];
V23 = RotToAngleAxis(R23);

% 2.4
disp(" ");
disp("Question 2.4:");
R24 = eye(3);
V24 = RotToAngleAxis(R24);



% 2.5
disp(" ");
disp("Question 2.5:");
R25 = [-1  0  0;
        0 -1  0;
        0  0  1];
V25 = RotToAngleAxis(R25);


% 2.6
disp(" ");
disp("Question 2.6:");
R26 = [-1  0  0;
        0  1  0;
        0  0  1];
V26 = RotToAngleAxis(R26);


%% 1.3 Euler to rot
clc;
clear;
% 3.2
disp(" ");
disp("Question 3.2:");
R32 = YPRToRot(0, 0, pi/2); 

% 3.3
disp(" ");
disp("Question 3.3:");
R33 = YPRToRot(deg2rad(60), 0, 0); 

% 3.4
disp(" ");
disp("Question 3.4:");
R34 = YPRToRot(pi/3, pi/2, pi/4);  

% 3.5
disp(" ");
disp("Question 3.5:");
R35 = YPRToRot(0, pi/2, -pi/12);   


%% 1.4 Rot to Euler
clc;
clear;
% 4.2
disp(" ");
disp("Question 4.2:");
R42 = [1  0  0;
       0  0 -1;
       0  1  0 ];
RotToYPR(R42);

% 4.3
disp(" ");
disp("Question 4.3:");
R43 = [1/2        -sqrt(3)/2   0;
       sqrt(3)/2   1/2         0;
       0           0           1 ];
RotToYPR(R43);


% 4.4
disp(" ");
disp("Question 4.4:");
R44 = [0          -sqrt(2)/2    sqrt(2)/2;
       1/2         sqrt(6)/4    sqrt(6)/4;
      -sqrt(3)/2    sqrt(2)/4    sqrt(2)/4 ];
RotToYPR(R44);


%% 1.5 Rot to angle-axis with eigenvectors
clc;
clear;
% 5.1
disp(" ");
disp("Question 5.1:");
R51 = [1  0  0;
       0  0 -1;
       0  1  0 ];
[h51,theta51] = RotToAngleAxis(R51);
RotToAngleAxisEigen(R51,theta51);

% 5.2
disp(" ");
disp("Question 5.2:");
R52 =(1/9)*[4  -4  -7;
            8   1   4;
           -1  -8   4];

[h52, theta52] = RotToAngleAxis(R52);
RotToAngleAxisEigen(R52, theta52);

%% 1.6 verifying and visualizing the hand computed transformation matrices 
clear;
clc;
PlotRefFrames;