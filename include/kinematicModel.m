%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian (can be either end-effector or tool, depending on last call)
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


        function bJ = getJacobianOfEndEffectorWrtBase(self)
            %% getJacobianOfEndEffectorWrtBase function
            % Computes the Jacobian for the END-EFFECTOR
            % Outputs:
            % bJ : 6xn Jacobian matrix mapping joint velocities to 
            %      end-effector velocity (linear and angular)
            
            bJ = zeros(6, self.gm.jointNumber);
            
            % End-effector position
            bTe = self.gm.getTransformWrtBase(self.gm.jointNumber);
            p_e = bTe(1:3, 4);
            
            for j = 1:self.gm.jointNumber
                bTj = self.gm.getTransformWrtBase(j);
                
                z_j = bTj(1:3, 3);
                p_j = bTj(1:3, 4);
                
                if self.gm.jointType(j) == 0  % revolute
                    bJ(1:3, j) = cross(z_j, p_e - p_j);
                    bJ(4:6, j) = z_j;
                else  % prismatic
                    bJ(1:3, j) = z_j;
                    bJ(4:6, j) = [0; 0; 0];
                end
            end
        end


        function bJ = getJacobianOfToolWrtBase(self)
            %% getJacobianOfToolWrtBase function
            % Computes the Jacobian for the TOOL
            % Outputs:
            % bJ : 6xn Jacobian matrix mapping joint velocities to 
            %      tool velocity (linear and angular)
            
            bJ = zeros(6, self.gm.jointNumber);
            
            % Tool position
            bTt = self.gm.getToolTransformWrtBase();
            p_t = bTt(1:3, 4);
            
            for j = 1:self.gm.jointNumber
                bTj = self.gm.getTransformWrtBase(j);
                
                z_j = bTj(1:3, 3);
                p_j = bTj(1:3, 4);
                
                if self.gm.jointType(j) == 0  % revolute
                    bJ(1:3, j) = cross(z_j, p_t - p_j);
                    bJ(4:6, j) = z_j;
                else  % prismatic
                    bJ(1:3, j) = z_j;
                    bJ(4:6, j) = [0; 0; 0];
                end
            end
        end


        function bJ = getJacobianOfJointWrtBase(self, i)
            %% getJacobianOfJointWrtBase function (GENERIC)
            % Computes the Jacobian for the i-th joint/frame
            % This is a generic function that can compute Jacobian for any frame
            % Inputs:
            % i : joint index (if i == jointNumber, it's the end-effector)
            % Outputs:
            % bJ : 6xn Jacobian matrix
            
            if i < 1 || i > self.gm.jointNumber
                error("Index out of bounds");
            end
            
            bJ = zeros(6, self.gm.jointNumber);
            
            % Position of frame i
            bTi = self.gm.getTransformWrtBase(i);
            p_i = bTi(1:3, 4);
            
            for j = 1:i
                bTj = self.gm.getTransformWrtBase(j);
                
                z_j = bTj(1:3, 3);
                p_j = bTj(1:3, 4);
                
                if self.gm.jointType(j) == 0  % revolute
                    bJ(1:3, j) = cross(z_j, p_i - p_j);
                    bJ(4:6, j) = z_j;
                else  % prismatic
                    bJ(1:3, j) = z_j;
                    bJ(4:6, j) = [0; 0; 0];
                end
            end
        end


        function updateJacobian(self)
            %% updateJacobian function
            % The function updates J to be the tool Jacobian
            % (used in the control loop)
            self.J = self.getJacobianOfToolWrtBase();
        end
    end
end