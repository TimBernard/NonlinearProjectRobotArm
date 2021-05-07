format compact
close all

%% Initialize Obstacles
Q1 = [0, 0.40, 0.40, 0; 0.15, 0.15, 0.30, 0.30];
Q2 = [0.60, 1, 1, 0.60; 0.15, 0.15, 0.30, 0.30];
O = {Q1, Q2};

%% Initial Inital state, goal state, and workspace
q_I = [-2*pi/3; 2*pi/3; 2*pi/3]; x_G = [1;1];
v_G = [0;0;0];
xmax = 1; ymax = 1;
workspace = [0, 0, xmax, xmax, 0; 0, ymax, ymax, 0, 0];

%% Initialize Control Parameters
S.m1 = 1;
S.m2 = 1;
S.m3 = 1;
S.l1 = .5;
S.l2 = .5;
S.l3 = .5;
S.lc1 = .25;
S.lc2 = .25;
S.lc3 = .25;
S.I1 = S.m1*S.l1/12;
S.I2 = S.m2*S.l2/12;
S.I3 = S.m3*S.l3/12;
S.g = 9.8;

S.kp = 2; S.kd = 1;

%% Plot Initial State
figure(1)
hold on
x_I = fwdKin(q_I);
plot(polyshape(Q1(1,:), Q1(2,:)))
plot(polyshape(Q2(1,:), Q2(2,:)))
plot(x_I(1,:), x_I(2,:), 'r-o')
plot(workspace(1,:), workspace(2,:), 'k')
axis([-0.5 1 -0.5 1])
hold off

%% Generate Path Tree
[path_indx, V, E, G, Q] = build_RRT(q_I, x_G, 5000, 0.01, O, xmax, ymax);
G(isinf(G)) = 0;
path = V(:,path_indx);

% Plot tree graph and Djikstra path
figure(2)
hold on
gplot(G, V', 'k-o')
plot(polyshape(Q1(1,:), Q1(2,:)))
plot(polyshape(Q2(1,:), Q2(2,:)))
plot(path(1,:), path(2,:), 'r-', 'LineWidth', 2)
hold off
xlim([0, xmax]); ylim([0, ymax]);

%% Animate Arm
figure(5)
hold on
plot(polyshape(Q1(1,:), Q1(2,:)))
plot(polyshape(Q2(1,:), Q2(2,:)))
plot(workspace(1,:), workspace(2,:), 'k')
axis([-0.5 1 -0.5 1])
for i = 1:length(path_indx)
    if i ~=1
        delete(h)
    end
    point = fwdKin(Q(:,path_indx(i)));
    h = plot(point(1,:),point(2,:), 'r-o');
    drawnow
end
hold off

%% Control Law
% Calculate the velocity between path points
% Vel = zeros(2,length(path));
% for i = 1:length(path)-1
%     Vel(:,i) = (Q(:,path_indx(i)) - Q(:,path_indx(i+1))) / norm(Q(:,path_indx(i)) - Q(:,path_indx(i+1)));
% end
% Vel(:,end) = v_G; Vel = 0.001*Vel;

% Use the control law to generate a smoother path between path points
full_pathQ = [];

q0 = [0;0;0;0;0;0]
qd = [1;1;1;0;0;0]

full_pathQ = arm_test(q0, qd, S)';

% Convert q in C-space to position in workspace
% full_pathSM = [];
% full_pathEE = [];
% for i = 1:length(full_pathQ)
%     full_pathSM = cat(3, full_pathSM, fwdKin(full_pathQ(1:2,i)));
%     full_pathEE = [full_pathEE, full_pathSM(:,end,end)];
% end

