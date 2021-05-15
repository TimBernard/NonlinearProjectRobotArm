function dq = Jacobian(q, dx)
    l1 = 0.5; l2 = 0.5; l3 = 0.5;
    
    t = zeros(1,3);
    for i = 1:length(q)
        t(i) = sum(q(1:i));
    end
    
    J = [-(l1*sin(t(1)) + l2*sin(t(2)) + l3*sin(t(3))), -(l2*sin(t(2)) + l3*sin(t(3))), -l3*sin(t(3));
           l1*cos(t(1)) + l2*cos(t(2)) + l3*cos(t(3)),    l2*cos(t(2)) + l3*cos(t(3)),   l3*cos(t(3))];
       
    dq = J'*inv(J*J')*dx;
end