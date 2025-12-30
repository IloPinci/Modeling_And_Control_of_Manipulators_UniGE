%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end

        
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control

            % define the gains as diagonal matrices
            K_A = self.k_a * eye(3);
            K_L = self.k_l * eye(3);

          
             %% for the end effector
            % n_joint = self.gm.jointNumber;
            % bTe = self.gm.getTransformWrtBase(n_joint);
            % 
            % err_pos = bTg(1:3, 4) - bTe(1:3, 4); %position error
            % 
            % Re = bTe(1:3, 1:3); %rotation matrices
            % Rg = bTg(1:3, 1:3);
            % 
            % ne = Re(:,1); ng = Rg(:, 1);    % unit vectors x
            % se = Re(:,2); sg = Rg(:, 2);    % unit vectors y
            % ae = Re(:,3); ag = Rg(:, 3);    % unit vectors z

            % error_orientation = 0.5 * (cross(ne, ng) + cross(se, sg) + cross(ae, ag)); % rotation error


            %% for the tool
            bTt = self.gm.getToolTransformWrtBase(); % get tool frame wrt base
    
            err_pos = bTg(1:3, 4) - bTt(1:3, 4); % position (linear) error

            %% Siciliano method - It is unstable in and around 180 degrees
            % nt = bRt(:,1); ng = bRg(:, 1); 
            % st = bRt(:,2); sg = bRg(:, 2);
            % at = bRt(:,3); ag = bRg(:, 3);
            % 
            % error_orientation = 0.5 * (cross(nt, ng) + cross(st, sg) + cross(at, ag));
            
            
            %% Angle Axis Method
            bRt = bTt(1:3, 1:3);    % rotation from base to tool
            bRg = bTg(1:3, 1:3);    % rotation from base to goal

            tRb = bRt';     % rotation from tool to base
            tRg = tRb * bRg;    % rotation from tool to goal

            % convert to the angle axis representation
            [h, theta] = RotToAngleAxis(tRg);

            % angular error in tool frame
            t_error_orientation = theta * h;

            % project to angular error of the tool in the base frame
            error_orientation = bRt * t_error_orientation;


            %% Common for the others
            
            % compute the desired velocities 
            desired_angular = K_A * error_orientation;
            desired_linear = K_L * err_pos;

            x_dot = [desired_linear; desired_angular];
        end
    end
end

