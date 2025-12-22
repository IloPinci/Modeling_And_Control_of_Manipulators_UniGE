%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType, eTt)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                self.eTt =  eTt;
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end



        function updateDirectGeometry(self, q)
            %% updateDirectGeometry function
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            %TO DO
            self.q = q(:);

            for i = 1:self.jointNumber
                T_static = self.iTj_0(:,:,i); % static transformation
                q_i = self.q(i);              % joint value

                if self.jointType(i) == 0
                    % rotational joint
                    c = cos(q_i);
                    s = sin(q_i);

                    T_actuation = [c -s 0 0;
                                   s  c 0 0;
                                   0  0 1 0;
                                   0  0 0 1];

                elseif self.jointType(i) == 1
                    % prismatic joint
                    T_actuation = [1 0 0 0;
                                   0 1 0 0;
                                   0 0 1 q_i;
                                   0 0 0 1];
                else
                    error('You have inputted a wrong joint type at %d: %d', i, self.jointType(i));
                end
                
                % apply joint actuation after the fixed geometry
                self.iTj(:, :, i) = T_static * T_actuation;
            end
        end




        function [bTk] = getTransformWrtBase(self,k)
            %% getTransformWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            %TO DO
            bTk = eye(4); % initialize
            for i = 1 : k
                 bTk = bTk * self.iTj(:, :, i);
            end
        end




         function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % outputs
            % bTt : transformation matrix from the manipulator base to the
            % tool

            %TO DO
            n_joints = self.jointNumber;

            b_T_e = self.getTransformWrtBase(n_joints); % we get the transform from base to ee
            bTt = b_T_e * self.eTt;  % then we multiply with the transform from ee to tool:  0Te * eTt = 0Tt
         end

    end
end


