function [isRotationMatrix] = IsRotationMatrix(R)

    isRotationMatrix = true;

    R_t = R';
    I = eye(3);
    c = (norm((R_t * R) + (-I)) < 1e-3);

    if ~c
        disp("The orthonormality condition is not met!");
        isRotationMatrix = false;
    end
    
    R_det = det(R);
    c = (abs(R_det - 1) < 1e-3);

    if ~c
        disp("The determinant condition is not met!");
        isRotationMatrix = false;
    end
end




    