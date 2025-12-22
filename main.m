%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';


%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition: we have a rotation matrix and a translation

% our rotation matrix is Yaw-Pitch-Roll so: R = RzRyRx

psi = pi/10;
theta = 0;
phi = pi/6;

Rz = [cos(psi) -sin(psi)  0;
      sin(psi) cos(psi)   0;
         0        0       1];  %yaw

Ry = [cos(theta)  0   sin(theta);
          0       1       0;
      -sin(theta) 0   cos(theta)]; %pitch

Rx = [ 1       0            0;
       0    cos(phi)   -sin(phi);
       0    sin(phi)    cos(phi)]; %roll

eRt = Rz*Ry*Rx;     % the rotation matrix
    
e_r_te = [0.3; 0.1; 0]; %translation vector  


eTt = [eRt,   e_r_te;
       0 0 0     1   ]; % the final transformation matrix




%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0, jointType, eTt);

% Update direct geoemtry given q0
gm.updateDirectGeometry(q);

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();

disp("eTt");
disp(eTt);
disp('bTt q = 0');
disp(bTt);





%% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = ...
bRg = ...
bTg = [bRg bOg;0 0 0 1]; 
disp('bTg')
disp(bTg)

% control proportional gain 
k_a = ...
k_l = ...

% Cartesian control initialization
cc = cartesianControl(....);

%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 

    % Get the cartesian error given an input goal frame

    % Update the jacobian matrix of the given model

    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    q_dot = ...

    % simulating the robot
    q = KinematicSimulation(....);
    
    pm.plotIter(gm, km, i, q_dot);

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

pm.plotFinalConfig(gm);
