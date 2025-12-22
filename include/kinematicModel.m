%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end





%% we change this function bc now we have to compute for the velocities of the tool
% we do not care for the joint velocity anymore. This is bc our jacobian
% needs to map joint velocities to the motion of the tool, and not the join
% motion. 
        function bJ = getJacobianOfJointWrtBase(self, i)

            if i < 1 || i > self.gm.jointNumber
                error("Index out of bounds");
            end
        
            bJ = zeros(6, self.gm.jointNumber);
        
            % tool position 
            bTt = self.gm.getToolTransformWrtBase();
            p_t = bTt(1:3,4);
        
            for j = 1:i
                bTj = self.gm.getTransformWrtBase(j);
        
                z_j = bTj(1:3,3);
                p_j = bTj(1:3,4);
        
                if self.gm.jointType(j) == 0  % revolute
                    bJ(1:3,j) = cross(z_j, p_t - p_j); % we replace with the position of the tool
                    bJ(4:6,j) = z_j;
                else                          % prismatic
                    bJ(1:3,j) = z_j;
                    bJ(4:6,j) = [0;0;0];
                end
            end
        end





        function updateJacobian(self)
        %% updateJacobian function
        % The function update:
        % - J: end-effector jacobian matrix
            % TO DO
             self.J = self.getJacobianOfJointWrtBase(self.gm.jointNumber);
        end
    end
end
