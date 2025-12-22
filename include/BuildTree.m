function [iTj_0] = BuildTree()

function rotation_z=Z(psi)

psi=deg2rad(psi);
rotation_z=[cos(psi) -sin(psi) 0;
            sin(psi) cos(psi) 0;
            0 0 1];
end

function rotation_y=Y(theta)

theta=deg2rad(theta);
rotation_y=[cos(theta) 0 sin(theta);
            0           1           0;
            -sin(theta) 0 cos(theta)];
end

function rotation_x=X(phi)

phi=deg2rad(phi);
rotation_x=[1      0       0;
            0 cos(phi) -sin(phi);
            0 sin(phi) cos(phi)];
end

ident=[0 0 0 1];

R_01=eye(3);
iTj_0(:, :, 1)=[R_01 [0 0 0.105]'; ident];

R_12=  Z(-90)*Y(-90);
iTj_0(:, :, 2)=[R_12 [0 0 0.110]'; ident];

R_23=Y(90)*Z(180);
iTj_0(:, :, 3)=[R_23 [0.100 0 0]'; ident];

R_34=Y(90)*Z(180);
iTj_0(:, :, 4)=[R_34 [0 0 0.325]'; ident];

R_45=Y(90)*Z(-90);
iTj_0(:, :, 5)=[R_45 [0.095 0 0]'; ident];

R_56=Z(-180);
iTj_0(:, :, 6)=[R_56 [0 0 0.095]'; ident];

R_67=Y(-90)*X(90);
iTj_0(:, :, 7)=[R_67 [0 0 0.355]'; ident];

end