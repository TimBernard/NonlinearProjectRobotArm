% Given an x,y position, find q1 and q2
function q = invKin(x)
    l1 = 0.5; l2 = 0.5;
    q2 = acos((x(1)^2 + x(2)^2 - l1^2 - l2^2)/(2*l1*l2));
    q1 = atan(x(2)/x(1))-atan((l2*sin(q2))/(l1+l2*cos(q2)));
    q = [q1;q2];
end