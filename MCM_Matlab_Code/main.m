%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
disp('TREE');
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
geometricModel = geometricModel(iTj_0,jointType);

%% Q1.3
disp("Q1.3");

qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4]; 
geometricModel.updateDirectGeometry(qi);

% (Base to End-Effector)
% Base is index 0. End-Effector is the last joint (7).
n_joints = geometricModel.jointNumber;
bTe = geometricModel.getTransformWrtBase(0, n_joints);
disp('Transformation Base -> End-Effector (bTe):');
disp(bTe);

% (Frame 6 to Frame 2)
T_6_2 = geometricModel.getTransformWrtBase(6, 2);
disp('Transformation Frame 6 -> Frame 2 (6T2):');
disp(T_6_2);

%% Q1.4 Simulation
disp('Q1.4');
% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(qi)
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(0, length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation ?
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/4, 0, -pi/4, 0, 0.18, pi/5];

%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples

    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(0, length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(0, j); 
        end
        pm.plotIter(bTi)
    end

end

pm.plotFinalConfig(bTi)

%% Q1.5
disp('Q1.5');
km = kinematicModel(geometricModel);
% Ensure model is at qf
geometricModel.updateDirectGeometry(qf'); 
J6 = km.getJacobianOfLinkWrtBase(6);
disp('Jacobian of Link 6 w.r.t Base (J6):');
disp(J6);

%% Q1.6
disp('Q1.6');
km.updateJacobian(); % Updates internal J for end-effector (link 7)
J_ee = km.J;
disp('Jacobian of End-Effector w.r.t Base (J_ee):');
disp(J_ee);

%% Q1.7
disp('Q1.7');
q_given = [0.7, -0.1, 1, -1, 0, 0.03, 1.3]';
q_dot_given = [0.9, 0.1, -0.2, 0.3, -0.8, 0.5, 0]';

%Update Geometry to q_given
geometricModel.updateDirectGeometry(q_given);

%Get Jacobian for End-Effector
km.updateJacobian();
J_base = km.J;

%Compute velocities in Base Frame
V_base = J_base * q_dot_given; % 6x1 vector [v_x; v_y; v_z; w_x; w_y; w_z]

%Get Rotation Matrix from Base to End-Effector
bTe_given = geometricModel.getTransformWrtBase(0, 7);
bRe = bTe_given(1:3, 1:3);

%Project velocities into End-Effector Frame
v_base_lin = V_base(1:3);
w_base_ang = V_base(4:6);

v_ee_lin = bRe' * v_base_lin;
w_ee_ang = bRe' * w_base_ang;

disp('Linear Velocity projected on EE frame (e_v_e/b):');
disp(v_ee_lin);
disp('Angular Velocity projected on EE frame (e_w_e/b):');
disp(w_ee_ang);



%Appendix Implementation

% %% Checking Jacobian (Link 6 and End-Effector)
% disp('%% Checking Jacobian (Link 6 and EE)');
% 
% km = kinematicModel(geometricModel);
% qf_col = qf(:);                      % force column vector
% geometricModel.updateDirectGeometry(qf_col);
% 
% % Check link 6
% disp('--- Check Jacobian for Link 6 ---');
% J6_num = km.checkJacobianNumerically(6, qf_col, 1e-6);
% 
% % Check end-effector (last link)
% disp('--- Check Jacobian for End-Effector (Link 7) ---');
% Jee_num = km.checkJacobianNumerically(geometricModel.jointNumber, qf_col, 1e-6);
% 
% %% Checks for Q1.7
% 
% %Check 1: Norm preservation under rotation
% fprintf('Check 1 (norm v):  ||v_b|| = %.6f, ||v_e|| = %.6f, diff = %.3e\n', ...
%     norm(v_base_lin), norm(v_ee_lin), abs(norm(v_base_lin) - norm(v_ee_lin)));
% 
% fprintf('Check 1 (norm w):  ||w_b|| = %.6f, ||w_e|| = %.6f, diff = %.3e\n', ...
%     norm(w_base_ang), norm(w_ee_ang), abs(norm(w_base_ang) - norm(w_ee_ang)));
% 
% %Check 2: Forward-kinematics finite difference of the twist in EE frame
% dt = 1e-6;
% 
% % Current transform
% geometricModel.updateDirectGeometry(q_given);
% T1 = geometricModel.getTransformWrtBase(0, 7);
% 
% % Next transform using q + qdot*dt
% q_next = q_given + q_dot_given * dt;
% geometricModel.updateDirectGeometry(q_next);
% T2 = geometricModel.getTransformWrtBase(0, 7);
% 
% % Relative motion expressed in EE frame at time 1
% T_rel = inv(T1) * T2;
% R_rel = T_rel(1:3,1:3);
% p_rel = T_rel(1:3,4);
% 
% % Angular velocity in EE frame (use your SO(3) log helper if you want)
% w_hat = logm(R_rel);
% w_fk_e = [w_hat(3,2); w_hat(1,3); w_hat(2,1)] / dt;
% 
% % Linear velocity in EE frame (approx)
% v_fk_e = p_rel / dt;
% 
% disp('Check 2 (FK) linear velocity in EE frame:');  disp(v_fk_e);
% disp('Check 2 (FK) angular velocity in EE frame:'); disp(w_fk_e);
% 
% disp('Check 2 difference (your projected - FK):');
% disp([v_ee_lin - v_fk_e; w_ee_ang - w_fk_e]);
% 
% % restore original config
% geometricModel.updateDirectGeometry(q_given);
