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

        function bJi = getJacobianOfJointWrtBase(self, i)
            %% getJacobianOfJointWrtBase function
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi
            
            %TO DO
             if i > self.gm.jointNumber || i < 0
                error("Index out of bounds");
            end

            bJi = zeros(6, self.gm.jointNumber);

            % transform of link i wrt base
            bTi = self.gm.getTransformWrtBase(0, i);
            p_target = bTi(1:3, 4);

            % iterate joints from base to link i
            for j = 1:i

                bTj = self.gm.getTransformWrtBase(0, j);
                z_j = bTj(1:3, 3);
                p_j = bTj(1:3, 4);

                p_diff = p_target - p_j;

                if self.gm.jointType(j) == 0      % revolute
                    bJi(1:3, j) = cross(z_j, p_diff);
                    bJi(4:6, j) = z_j;

                elseif self.gm.jointType(j) == 1  % prismatic
                    bJi(1:3, j) = z_j;
                    bJi(4:6, j) = [0;0;0];
                end
            end
        end





        function updateJacobian(self)
        %% updateJacobian function
        % The function update:
        % - J: end-effector jacobian matrix
            % TO DO
             self.J = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
        end
    end
end
