function [h,theta] = RotToAngleAxis(R)
    c1 = IsRotationMatrix(R);
    R_tr = trace(R);

    if ~c1
        disp("This is not a valid rotation matrix. Provide a new one!");
        theta = NaN;
        h = [NaN; NaN; NaN];
        return
    else
        theta = acos((R_tr - 1) / 2);
    end
    

    if abs(theta) < 1e-3 
        disp("Since θ = 0, the vector h is arbitrary!");
        h = [1; 0; 0]; % we decide a vector for h
       
    elseif abs(theta-pi) < 1e-3 
        I = eye(3);
        hh_t = (R + I)/2;

        hx2 = hh_t(1, 1);
        hy2 = hh_t(2, 2);
        hz2 = hh_t(3, 3);

        hx = sqrt(hx2);
        hy = sqrt(hy2);
        hz = sqrt(hz2);

        %determine the sign
        if abs(hx) > 1e-5
            if (hx * hy) * sign(R(1, 2)) < 0
                hy = -hy;
            end
            if (hx * hz) * sign(R(1, 2)) < 0
                hz = -hz;
            end
        elseif abs(hy) > 1e-5
            if (hx * hz) * sign(R(1, 2)) < 0
                hz = -hz;
            end
        end

        h = [hx; hy; hz];

    else
        anti_symmetric = (R - R') / 2;
        h = vex(anti_symmetric) / sin(theta);

    end
end

 
function a = vex(S_a)
    a = [S_a(3, 2); S_a(1, 3); S_a(2, 1)];
end