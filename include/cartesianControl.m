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

            % we need to compute again the error since it is a function
            n_joint = self.gm.jointNumber;

            %% for the end effector
            % bTe = self.gm.getTransformWrtBase(n_joint);
            % 
            % e_pos = bTg(1:3, 4) - bTe(1:3, 4); %position error
            % 
            % Re = bTe(1:3, 1:3); %rotation matrices
            % Rg = bTg(1:3, 1:3);
            % 
            % ne = Re(:,1); ng = Rg(:, 1);    % unit vectors x
            % se = Re(:,2); sg = Rg(:, 2);    % unit vectors y
            % ae = Re(:,3); ag = Rg(:, 3);    % unit vectors z

            % e_orient = 0.5 * (cross(ne, ng) + cross(se, sg) + cross(ae, ag)); % rotation error


            %% for the tool
            bTt = self.gm.getToolTransformWrtBase(); 
    
            e_pos = bTg(1:3, 4) - bTt(1:3, 4); 

            Rt = bTt(1:3, 1:3);
            Rg = bTg(1:3, 1:3);

            nt = Rt(:,1); ng = Rg(:, 1); 
            st = Rt(:,2); sg = Rg(:, 2);
            at = Rt(:,3); ag = Rg(:, 3);

            e_orient = 0.5 * (cross(nt, ng) + cross(st, sg) + cross(at, ag));
            
            %% Common for the others
            
            desired_angular = K_A * e_orient;
            desired_linear = K_L * e_pos;

            x_dot = [desired_linear; desired_angular];
        end
    end
end

