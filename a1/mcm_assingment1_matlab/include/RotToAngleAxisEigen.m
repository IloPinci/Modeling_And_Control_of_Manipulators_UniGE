function [h_eigen,theta_eigen] = RotToAnlgeAxisEigen(R,theta)
% Given a rotation matrix this function 
% should output the equivalent angle-axis representation values
% based on the Eigenvector of the Rotation Matrix
% Input: R: Rotation Matrix, theta: theta computed by RotToAngleAxis
% Output: h_eigen: h using eigen values, theta_eigen: identical to theta

h_eigen = [NaN;NaN;NaN];
if ~IsRotationMatrix(R)
    disp("This is not a valid Rotation Matrix. Provide a new one!");
    disp("The Angle Axis using RotToAngleAxis is:")
    disp(h_eigen);
    disp(theta);
    return
end

[V,D] = eig(R); %Where we have eigenvalues in diagonal of D and eigenvectors in columns of V
i = find(abs(diag(D)-1)<1e-3);

if isempty(i)
    disp("Matrix has no eigenvalue 1"); 
    disp("The Angle Axis using RotToAngleAxis is:")
    disp(h_eigen);
    disp(theta);
    return
end

h_eig = V(:,i);
h_eigen = h_eig/norm(h_eig);
theta_eigen = theta;

disp("The Angle Axis using RotToAngleAxisEigen is:")
disp(h_eigen);
disp(theta_eigen);

end