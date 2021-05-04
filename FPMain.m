format compact
close all

%% Test build_PRM
Q1 = [0, 0.35, 0.35, 0; 0.15, 0.15, 0.30, 0.30];
Q2 = [0.65, 1, 1, 0.65; 0.15, 0.15, 0.30, 0.30];
O = {Q1, Q2};

q_I = [-0.49*pi; 0.98*pi]; x_G = [0.5;0.5];
v_I = (10^-5) * [1;1]; v_G = [0;0];
xmax = 1; ymax = 1;

figure(1)
x_I = fwdKin(q_I);
plot(x_I(1,:), x_I(2,:))
axis([-2 2 -2 2])

%% Generate Path Tree
[path_indx, V, E, G, Q] = build_RRT(q_I, x_G, 5000, 0.01, O, xmax, ymax);
G(isinf(G)) = 0;
path = V(:,path_indx);

figure(2)
hold on
gplot(G, V', 'k-o')
plot(polyshape(Q1(1,:), Q1(2,:)))
plot(polyshape(Q2(1,:), Q2(2,:)))
plot(path(1,:), path(2,:), 'r-', 'LineWidth', 2)
hold off
xlim([0, xmax]); ylim([0, ymax]);

%% Animate Arm;
Vel = zeros(2,length(path));
for i = 1:length(path)-1
    Vel(:,i) = (Q(:,path_indx(i)) - Q(:,path_indx(i+1))) / norm(Q(:,path_indx(i)) - Q(:,path_indx(i+1)));
end
Vel(:,end) = v_G; Vel = 0.0001*Vel;

full_pathQ = [];
for i = 1:length(path_indx)-1
    full_pathQ = [full_pathQ, arm_test([Q(:,path_indx(i)); Vel(:,i)], [Q(:,path_indx(i+1)); Vel(:,i+1)])'];
end

full_pathSM = [];
full_pathEE = [];
for i = 1:length(full_pathQ)
    full_pathSM = cat(3, full_pathSM, fwdKin(full_pathQ(1:2,i)));
    full_pathEE = [full_pathEE, full_pathSM(:,end,end)];
end

figure(3)
hold on
plot(polyshape(Q1(1,:), Q1(2,:)))
plot(polyshape(Q2(1,:), Q2(2,:)))
plot(full_pathEE(1,:), full_pathEE(2,:))
hold off
axis([0 1 0 1])

figure(4)
hold on
plot(polyshape(Q1(1,:), Q1(2,:)))
plot(polyshape(Q2(1,:), Q2(2,:)))
axis([-0.5 1 -0.5 1])
for i = 1:length(full_pathSM)
    if i ~=1
        delete(h)
    end
    h = plot(full_pathSM(1,:,i),full_pathSM(2,:,i), 'r-o');
    drawnow
end
hold off