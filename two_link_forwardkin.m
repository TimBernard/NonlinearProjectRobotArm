function pose = two_link_forwardkin(q,S)
% Forward Kinematics for two-link robot manipulator 
% 
% Input: q, joint vector (2x1 array)
% Output: pose, SE2 forward kinematics (3x3 matrix)

px = S.l1*cos(q(1)) + S.l2*cos(q(1) + q(2));
py = S.l1*sin(q(1)) + S.l2*sin(q(1) + q(2));
p = [px; py];
R = [cos(q(1)+q(2)), - sin(q(1)+q(2)); sin(q(1)+q(2)), cos(q(1)+q(2))];
pose = [R,p; 0, 0, 1];
end

