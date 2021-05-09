function pose = elbow_forwardkin(q,S)
% Forward Kinematics for three-link (elbow) manipulator
%   from MLS Example 3.2 (pg. 89)
%
% Input: q, joint vector (3x1 array)
% Output: pose, SE3 forward kinematics (4x4 matrix) 

twist1 = [0; 0; 0; 0; 0; 1];
twist2 = [0; -S.l0; 0; -1; 0; 0];
twist3 = [0; -S.l0; S.l1; -1; 0; 0];

exp1 = expm(wedge(twist1)*q(1));
exp2 = expm(wedge(twist2)*q(2));
exp3 = expm(wedge(twist3)*q(3));
gst0 = [eye(3), [0; (S.l1+S.l2); S.l0];
        zeros(1,3), 1];
pose = exp1*exp2*exp3*gst0;
end

function [w_hat] = skew3(w)
% Function: Acts as wedge/skew operatior for R^3 --> so(3)
% Input:    w, 3x1 vector 
% Output:   w_hat, 3x3 skew symmetric matrix 
w_hat = [0 -w(3) w(2);
         w(3) 0 -w(1);
         -w(2) w(1) 0];
end

function [xi_hat] = wedge(xi)
% Function: Acts as wedge/skew operatior for R^6 --> se(3)
% Input:    xi, twist coordinates {v,w} (6x1 vector)
% Output:   xi_hat, twist 4x4 skew symmetric matrix 

w_hat = skew3(xi(4:6));
v = xi(1:3);
xi_hat = [w_hat, v; zeros(1,4)];
end