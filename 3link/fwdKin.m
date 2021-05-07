% Given q1 and q2, find the position of each link
function x = fwdKin(q)
    l1 = 0.5; l2 = 0.5; l3 = 0.5;
    x(:,1) = [0;0];
    x(:,2) = [l1*cos(q(1)); l2*sin(q(1))];
    x(:,3) = [l1*cos(q(1)) + l2*cos(q(1)+q(2)); l1*sin(q(1)) + l2*sin(q(1)+q(2))];
    x(:,4) = [l1*cos(q(1)) + l2*cos(q(1)+q(2)) + l3*cos(q(1)+q(2)+q(3)); l1*sin(q(1)) + l2*sin(q(1)+q(2)) + l3*sin(q(1)+q(2)+q(3))];
end