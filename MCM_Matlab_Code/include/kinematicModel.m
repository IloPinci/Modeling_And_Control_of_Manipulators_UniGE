%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    properties
        gm         % geometric model
        J          % Full Jacobian (EE)
    end

    methods
        %% Constructor
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)');
            end
        end


        %% Jacobian of link i wrt base
        function bJi = getJacobianOfLinkWrtBase(self, i)

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


        %% Update EE Jacobian
        function updateJacobian(self)
            self.J = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
        end


        function J_num = checkJacobianNumerically(self, linkNumber, q, delta)
            if nargin < 4
                delta = 1e-6;
            end

            % Ensure geometry consistent
            self.gm.updateDirectGeometry(q);

            % Analytical Jacobian
            J_analytical = self.getJacobianOfLinkWrtBase(linkNumber);

            % Numerical Jacobian
            J_num = zeros(6, self.gm.jointNumber);

            % Nominal transform
            T_nom = self.gm.getTransformWrtBase(0, linkNumber);
            p_nom = T_nom(1:3,4);
            R_nom = T_nom(1:3,1:3);

            for j = 1:self.gm.jointNumber

                % Perturb joint j
                q_pert = q;
                q_pert(j) = q_pert(j) + delta;

                self.gm.updateDirectGeometry(q_pert);

                % Perturbed transform
                T_pert = self.gm.getTransformWrtBase(0, linkNumber);
                p_pert = T_pert(1:3,4);
                R_pert = T_pert(1:3,1:3);

                % Linear velocity derivative
                dp = (p_pert - p_nom)/delta;

                % Angular velocity DERIVATIVE IN BASE FRAME
                dR = R_pert * R_nom';      % relative rotation in base frame
                w_hat = logm(dR);
                w = [w_hat(3,2); w_hat(1,3); w_hat(2,1)] / delta;

                % Fill correct column
                J_num(:,j) = [dp; w];

            end

            % Restore original configuration
            self.gm.updateDirectGeometry(q);

            % Print
            disp('Analytical Jacobian:'), disp(J_analytical)
            disp('Numerical Jacobian:'), disp(J_num)
            disp('Difference:'), disp(J_analytical - J_num)

        end


        %% Helper: Logarithm of SO(3)
        function L = logm_SO3(~, R)
            % Numerically stable logarithm for SO(3)
            theta = acos( max(-1,min(1,(trace(R)-1)/2)) );
            if abs(theta) < 1e-8
                L = zeros(3);
                return;
            end
            L = theta/(2*sin(theta)) * (R - R');
        end

        %% Helper: vee operator
        function v = vee(~, S)
            v = [ S(3,2); S(1,3); S(2,1) ];
        end

    end
end
