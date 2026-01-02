% function [h,theta] = RotToAngleAxis(R)
%     c1 = IsRotationMatrix(R);
%     R_tr = trace(R);
% 
%     if ~c1
%         disp("This is not a valid rotation matrix. Provide a new one!");
%         theta = NaN;
%         h = [NaN; NaN; NaN];
%         return
%     else
%         theta = acos((R_tr - 1) / 2);
%     end
% 
% 
%     if abs(theta) < 1e-3 
%         disp("Since θ = 0, the vector h is arbitrary!");
%         h = [1; 0; 0]; % we decide a vector for h
% 
%     elseif abs(theta-pi) < 1e-3 
%         I = eye(3);
%         hh_t = (R + I)/2;
% 
%         hx2 = hh_t(1, 1);
%         hy2 = hh_t(2, 2);
%         hz2 = hh_t(3, 3);
% 
%         hx = sqrt(hx2);
%         hy = sqrt(hy2);
%         hz = sqrt(hz2);
% 
%         %determine the sign
%         if abs(hx) > 1e-5
%             if (hx * hy) * sign(R(1, 2)) < 0
%                 hy = -hy;
%             end
%             if (hx * hz) * sign(R(1, 2)) < 0
%                 hz = -hz;
%             end
%         elseif abs(hy) > 1e-5
%             if (hx * hz) * sign(R(1, 2)) < 0
%                 hz = -hz;
%             end
%         end
% 
%         h = [hx; hy; hz];
% 
%     else
%         anti_symmetric = (R - R') / 2;
%         h = vex(anti_symmetric) / sin(theta);
% 
%     end
% end
% 
% 
% function a = vex(S_a)
%     a = [S_a(3, 2); S_a(1, 3); S_a(2, 1)];
% end

function [h, theta] = RotToAngleAxis(R)
    if ~IsRotationMatrix(R)
        theta = NaN;
        h = [NaN; NaN; NaN];
        return
    end

    % Robust theta from trace (clamp for numerical safety)
    x = (trace(R) - 1) / 2;
    x = min(1, max(-1, x));
    theta = acos(x);

    eps_theta = 1e-6;   % tighter than 1e-3 for control
    eps_pi    = 1e-6;

    if theta < eps_theta
        % Near zero rotation: error vector should be zero
        theta = 0;
        h = [0; 0; 0];
        return
    end

    if abs(theta - pi) < eps_pi
        % Near pi: use (R + I)/2 to get axis components squared
        A = (R + eye(3)) / 2;

        % Choose the largest diagonal for stability
        [~, idx] = max([A(1,1), A(2,2), A(3,3)]);

        h = zeros(3,1);
        h(idx) = sqrt(max(A(idx,idx),0));

        % Compute other components from off-diagonals
        if idx == 1
            if h(1) > 0
                h(2) = A(1,2) / h(1);
                h(3) = A(1,3) / h(1);
            end
        elseif idx == 2
            if h(2) > 0
                h(1) = A(1,2) / h(2);
                h(3) = A(2,3) / h(2);
            end
        else
            if h(3) > 0
                h(1) = A(1,3) / h(3);
                h(2) = A(2,3) / h(3);
            end
        end

        % Normalize (guard)
        n = norm(h);
        if n > 0
            h = h / n;
        end
        return
    end

    % General case
    S = (R - R') / 2;
    h = vex(S) / sin(theta);

    % Normalize (guard)
    n = norm(h);
    if n > 0
        h = h / n;
    end
end

function a = vex(S)
    a = [S(3,2); S(1,3); S(2,1)];
end
