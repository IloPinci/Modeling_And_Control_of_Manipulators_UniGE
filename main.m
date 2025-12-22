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

psi_t = pi/10;
theta_t = 0;
phi_t = pi/6;

Rz = [cos(psi_t) -sin(psi_t)  0;
      sin(psi_t) cos(psi_t)   0;
         0        0       1];  %yaw

Ry = [cos(theta_t)  0   sin(theta_t);
          0       1       0;
      -sin(theta_t) 0   cos(theta_t)]; %pitch

Rx = [ 1       0            0;
       0    cos(phi_t)   -sin(phi_t);
       0    sin(phi_t)    cos(phi_t)]; %roll

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
bOg = [0.2; -0.7; 0.3]; % the translation from base to goal

% for the rotation we need to redefine the angles that show rotation from
% base to goal
psi_g = 0;
theta_g = 1.57;
phi_g = 0;


Rz = [cos(psi_g) -sin(psi_g)  0;
      sin(psi_g) cos(psi_g)   0;
         0        0       1];  %yaw

Ry = [cos(theta_g)  0   sin(theta_g);
          0       1       0;
      -sin(theta_g) 0   cos(theta_g)]; %pitch

Rx = [ 1       0            0;
       0    cos(phi_g)   -sin(phi_g);
       0    sin(phi_g)    cos(phi_g)]; %roll


bRg = Rz * Ry * Rx;


bTg = [bRg bOg;
       0 0 0 1]; 

disp('bTg')
disp(bTg)


%% Ex 2.1 Cartesian error

% we need to find the cartesian error. It is a 6x1 that has the orientation error
% and the position error. 

% The ladder is expressed as: 
% e = xg - xe (the position of the goal - the position of the ee)

n_joint = gm.jointNumber;
bTe = gm.getTransformWrtBase(n_joint);

e_position = bOg - bTe(1:3, 4);


% since we found the position error, we need to find the orientation error
% but to find this orentation error we have three methods. The easiest one
% is the one that does the difference between euler angles. However this is
% not very robust, due to the fact that it is prone to singularities. As a
% result we will use the rotation matrix method. 

Re = bTe(1:3, 1:3); % we get the rotation matrix

% we extract the unit vectors that form the rotation matrices
ne = Re(:, 1); ng = bRg(:, 1); % we get the normal vector
se = Re(:, 2); sg = bRg(:, 2); % we get the sliding vector
ae = Re(:, 3); ag = bRg(:, 3); % we get the approach vector

% we calculate the geometric form off the orientation error
e_orientation = 0.5 * ( cross(ne, ng) + cross(se, sg) + cross(ae, ag) );

cartesian_error = [e_position; e_orientation];

disp("Cartesian error: e = [e_position, e_orientation]'")
disp(cartesian_error)


%% Ex 2.2 Desired angular the linear velocities

% control proportional gain
k_a = 0.8;
k_l = 0.8;

% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

% usage of the cc
result = cc.getCartesianReference(bTg);

desired_linear  = result(1:3);
desired_angular = result(4:6);

disp("Desired angular velocity:")
disp(desired_angular)

disp("Desired linear velocity:")
disp(desired_linear)



%% Ex 2.3 Compute the desired joint velocities

% we chose to do it with the minimum norm solution to inverse kinematic
% problem
J = km.getJacobianOfJointWrtBase(n_joint);

% we do it with two methods. One is the pseudo inverse of matlab. The
% second is the right pseudoinverse done manually
q_dot = pinv(J) * [desired_linear; desired_angular];
q_dot1 = (J' * inv(J*J')) * [desired_linear; desired_angular];

disp("q_dot:")
disp(q_dot)

disp("q_dot1:")
disp(q_dot1)


%% Ex 2.4

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
    gm.updateDirectGeometry(q);

    % Get the cartesian error given an input goal frame
    x_dot = cc.getCartesianReference(bTg);

    % Update the jacobian matrix of the given model
    J = km.getJacobianOfJointWrtBase(gm.jointNumber);

    %% INVERSE KINEMATICS
    % Compute desired joint velocities 
    q_dot = pinv(J) * x_dot;


    % simulating the robot
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    pm.plotIter(gm, km, i, q_dot);

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end

end

pm.plotFinalConfig(gm);



%% Ex 2.5 End-effector and tool velocities (wrt base, expressed in base)

% End-effector velocity
J = km.getJacobianOfJointWrtBase(n_joint);
x_dot_e = J * q_dot;

v_e     = x_dot_e(1:3);
omega_e = x_dot_e(4:6);

% Vector from EE to tool expressed in base
bTe = gm.getTransformWrtBase(n_joint);
bRe = bTe(1:3,1:3);
b_r_et = bRe * e_r_te;

% Tool velocity
omega_t = omega_e;
v_t     = v_e + cross(omega_e, b_r_et);

x_dot_t = [v_t; omega_t];

disp("End-effector velocity [v; omega] (base frame):")
disp(x_dot_e)

disp("Tool velocity [v; omega] (base frame):")
disp(x_dot_t)

