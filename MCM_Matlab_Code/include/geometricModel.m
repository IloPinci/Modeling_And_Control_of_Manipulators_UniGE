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
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end

        %% Ex 2
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;
            %
            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            % normalize q to column vector (borrowed from Charbel)
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

        %% Ex 3
        function [bTk] = getTransformWrtBase(self, start_idx, end_idx)
            %% GetTransformatioWrtBase function
            % Inputs :
            % start_idx, end_idx: indices of start and end frames
            % outputs
            % bTk : transformation matrix from frame <start_idx> to <end_idx>
            % in the configuration identified by iTj.

            % check parameters
            if (start_idx > self.jointNumber || start_idx < 0) || ...
               (end_idx   > self.jointNumber || end_idx   < 0)
                error('The provided indexes are out of bounds');
            end

            bTk = eye(4); % initialize

            if start_idx == end_idx
                return;
            end

            if start_idx < end_idx
                % forward product
                for i = (start_idx + 1) : end_idx
                    bTk = bTk * self.iTj(:, :, i);
                end

            elseif start_idx > end_idx
                % compute forward from end to start, then invert
                for i = (end_idx + 1) : start_idx
                    bTk = bTk * self.iTj(:, :, i);
                end
                bTk = bTk \ eye(4);
            end
        end

        % Note: The calculation starts from index s+1 because the transform
        % from b to 1 is at index 1, not 0 (MATLAB indexing).
        % So: 0T2 = 0T1 (index 1) * 1T2 (index 2).
    end
end
