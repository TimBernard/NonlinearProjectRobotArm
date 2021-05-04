close all 

Q1 = [0, 0.25, 0.25, 0; 0.15, 0.15, 0.30, 0.30];
Q2 = [0.75, 1, 1, 0.75; 0.15, 0.15, 0.30, 0.30];
O = {Q1, Q2};

V = [0;0];
for i = 1:10000
    x_rand = [xmax*rand(); ymax*rand()];
    q_rand = invKin(x_rand);
    if isnocollision([x_rand, x_rand], O) && isNoChainCollision(fwdKin(q_rand), O)
        V = [V, x_rand];
    end
end

hold on
plot(polyshape(Q1(1,:), Q1(2,:)))
plot(polyshape(Q2(1,:), Q2(2,:)))
scatter(V(1,:),  V(2,:))
hold off

function b = isnocollision(S, O)
    flag = 1:length(O);
    for i = 1:length(O)
        flag(i) = isintersect_linepolygon(S, O{i});
    end
    
    % Check to see if S intersects with any O
    b = all(~flag);
end