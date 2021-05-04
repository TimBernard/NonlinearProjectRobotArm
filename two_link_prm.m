% Two-link robot PRM 
clear variables; clc; close all;
%% PATH GENERATION
% Use PRM to find collision-free path from start to goal
N = 500; % Number of random samples 

% Generate (random collision free) vertices
V = make_vertices(N);
% plot(V(1,:),V(2,:),'.')

% Create edges using knn with vertices 
E = make_edges(V);

% Organize edges into (N,2) matrix, remove duplicates
E = E';
E = unique(sort(E,2),'rows');

% Define desired start and end states 
% initial state with high-velocity
x0 = [0; 0; 2; 5];
q0 = x0(1:2);

% desired state
xf = [pi; 3*pi/2; 0; 0];
qf = xf(1:2);

% Add new init and final states to set 
V = [V, q0, qf];

% Find path 
path = search_path(V,E,q0,qf);

% Display path 
figure(1)
hold on 
V_path = V(:,path);
plot(V(1,:),V(2,:),'.');
plot(V_path(1,:),V_path(2,:),'-*r');
legend('Valid Configurations','Chosen Path');
legend('show');
xlabel('q_1')
ylabel('q_2')
title('Probabilistic Roadmap')
hold off

% Desired path 
q_path = V_path

%% DYNAMICS SIMULATION 
% Generate trajectories connecting each of the desired configurations 
% and track them 

% model parameters
S.m1 = 1;
S.m2 = 1;
S.l1 = .5;
S.l2 = .5;
S.lc1 = .25;
S.lc2 = .25;
S.I1 = S.m1*S.l1/12;
S.I2 = S.m2*S.l2/12;
S.g = 9.8;

% gains 
S.kp = 20; S.kd = 10; 

% Perturbed start configuration 
q_path(:,1) = q_path(:,1) + [0.15; 0.15]; 

% Generate trajectories and controls while plotting 
figure(2)
hold on
x_previous = [q_path(:,1); 2; 5];
for i=1:size(q_path,2)-1

    % Start from end of previous trajectory 
    x_start = x_previous;
    x_end = [q_path(:,i+1); 1; 1];
    [x_plan,xs] = plan_and_track(x_start,x_end,S);
    
    % Plot desired trajectory 
    p1 = plot(x_plan(1,:), x_plan(2,:), '-r');
    
    % Plot exectued trajectory 
    p2 = plot(xs(:,1), xs(:,2), '-b');
    
    x_previous = xs(end,:)';
end
    
% Mark position of start point 
p3 = plot(q_path(1,1),q_path(2,1),'*b','Markersize',20); 

% Mark position of goal point 
p4 = plot(qf(1),qf(2),'*g','Markersize',20); 

legend([p1, p2, p3, p4],{'desired','executed','start','goal'})
legend('show');
xlabel('q_1')
ylabel('q_2')
title('2D Manipulator trajectory routine')
hold off 

%-------------------- PRM/ Graph functions --------------------%
function V = make_vertices(N)
% Randomly (uniformly) generate N collision-free configurations 
% Input:  
%   N, number of vertices (positive int)
% Output: 
%   V, set of collision-free vertices (2xN array) 

V = zeros(2,N);
 count = 1; 
while count-1 < N
    q = 2*pi*rand(2,1);
    if(~state_collides(q))
        V(:,count) = q;
        count = count + 1;
    end       
end
end

function E = make_edges(V) 
% Create an array of edges (qa,qb)
% Input: 
%   V, set of collision-free vertices (2xN array) 
% Output: 
%   E, set of Edges 

E = zeros(2,5000); % Allocate enough space for edges 
k = 20; % Number of nearest neighbors for each vertex 
step = 0.1; % Step size between 
count = 1; % Keep track of how many edges are added 

% Make edges by looping through each vertex 
for i=1:size(V,2) 
    q = V(:,i);
    nn_idx = get_nn(q,V,k);
    
    % Try to create an edge with each of the nearest neighbbors 
    for j=1:k
        q_prime = V(:,nn_idx(j));
        
        % Continue only if straight line between vertices is collision-free
        % to reduce chance of collision during trajectory-generation 
        if(~straight_edge_collides(q,q_prime,step))
            edge = [i; nn_idx(j)];
            E(:,count) = edge;
            count = count + 1;              
        end 
    end 
end 
E = E(:,any(E));
end

function idx = search_departability(V,q_qoal)
% Find vertex that connects to q_goal without collisions
% Input: 
%   V, set of collision-free vertices (2xN array)
%   q, q_goal (2x1 array) 
% Output: 
%   idx, index of best vertex 

k = 20; % Number of nearest neighbors for each vertex 
step = 0.1; % Step size between 
nn_idx = get_nn(q_qoal,V,k);
for j=1:k
    q_prime = V(:,nn_idx(j));
    if(~straight_edge_collides(q_qoal,q_prime,0.1))
        idx = nn_idx(j);
        return;
    end 
end 
idx = -1; % if fails to find departable vertex  
end

function idx = search_accessibility(V,q_start)
% Find vertex that connects to q_goal without collisions
% Input: 
%   V, set of collision-free vertices (2xN array)
%   q, q_goal (2x1 array) 
% Output: 
%   idx, index of best vertex 

k = 20; % Number of nearest neighbors for each vertex 
step = 0.1; % Step size between 
nn_idx = get_nn(q_start,V,k);
for j=1:k
    q_prime = V(:,nn_idx(j));
    if(~straight_edge_collides(q_start,q_prime,0.1))
        idx = nn_idx(j);
        return;
    end 
end 
idx = -1; % if fails to find accessible vertex  
end

function path = search_path(V,E,q0,qf)
% Generate a path from q0 to qf 
% Inputs: 
%   V, set of collision-free vertices (2xN array)
%   E, edges
%   q0, start configuration
%   qf, final configuration
% Outputs: 
%   path, array of vertex indices from start to finish 

% Set indices of start/goal and find access and departure vertices
q0_idx = length(V)-1;
qf_idx = length(V);
access_idx = search_accessibility(V,q0);
depart_idx = search_departability(V,qf); 

% Add access and departure edges to set of edges 
E = [E; [q0_idx, access_idx]; [qf_idx, depart_idx]];

% Create graph and find shortest path 
G = graph(E(:,1),E(:,2));
path =shortestpath(G,q0_idx,qf_idx,'Method','unweighted');
end
                    
function nn_idx = get_nn(q,V,k) 
% Find the k-nearest neighbors to vertex 
% Input: 
%   q, 2x1 joint vector (2x1 array) 
%   V, set of vertices (2xN array) 
%   k, number of nearest neighbors (positive int)
% Output: 
%   nearest, k-nearest neighbors (kx1 array) 

distances = sqrt(sum(power(V-q,2),1));
[distances_sorted,idx] = sort(distances);
idx = idx(2:k+1);
nn_idx = idx; 
end

function bool = state_collides(q)
% Return whether configuration is in collision or not 
% Input: q, 2x1 joint vector (2x1 array) 
% Output: boolean, true if causes collision

% TODO
r = rand;
if r < 0.1
    bool = true;
else
    bool = false;
end
end

function bool = straight_edge_collides(qa,qb,step)
% Return whether configuration is in collision or not 
% Input: (qa,qb), pair of points/ edge
%         step
% Output: boolean, true if causes collision 
for t=0:step:1
    if(state_collides(interpolate(qa,qb,t)))
        bool=true;
        return;
    end
end
bool=false;
end

function qt = interpolate(qa,qb,t)
% Interpolate between configurations 
% Input: (qa,qb), points to interpolate between 
% Output: qt, point inbetween  
qt = (1.0-t)*qa + t*qb;
end

function [x_plan,xs] = plan_and_track(x1,x2,S)
% Generate and track the desired trajectory between two configurations 
% Inputs: 
%   x1, first state (4x1 array)
%   x2, second state (4x1 array) 
%   S, arm parameters (Struct) 
% Outputs:
%   x_plan, desired trajectory 
%   xs, executed trajectory

% This should take in two neighboring states (from the graph-generated path) 
% and then return the desired trajectory and the actual tracked trajectory 

T = 0.25; % simulation time

% boundary conditions in flat output space 
y1 = arm_h(x1);
y2 = arm_h(x2);
dy1 = [x1(3),x1(4)]';  % desired starting velocity
dy2 = [x2(3),x2(4)]'; % desired end velocity

% compute path coefficients
A = poly3_coeff(y1, dy1, y2, dy2, T);
S.A = A;

% desired path
x_plan = A*poly3([0:.01:T]);

[ts, xs] = ode45(@arm_ode, [0 T], x1, [], S);
end

function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
A = Y*inv(L);
end

function y = arm_h(x)
% output function
y = x(1:2);
end

function f = poly3(t)
f = [t.^3; t.^2; t; ones(size(t))];
end

function f = dpoly3(t)
f = [3*t.^2; 2*t; ones(size(t)); zeros(size(t))];
end

function f = d2poly3(t)
f = [6*t; 2; zeros(size(t)); zeros(size(t))];
end

function u = arm_ctrl(t, x, S)
% standard computed torque law
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% desired trajectory and controls
qd = yd;
vd = dyd;
ad = d2yd;

% current
q = x(1:2);
v = x(3:4);

[M, C, N] = arm_dyn(t, x, S);

%(For when desired state and acceleration are explicitly set) 
% u = M*(S.ad - S.kp*(q - qd) - S.kd*(v - vd)) + C*v + N;

% To follow generated trajectory 
ud = M*(ad) + C*v + N;
u = ud + M*(-S.kp*(q-qd) -S.kd*(v-vd));
end

function dx = arm_ode(t, x, S)
% the ODE for the arm

v = x(3:4);

[M, C, N] = arm_dyn(t, x, S);

u = arm_ctrl(t, x, S);

dx = [v;
      inv(M)*(u - C*v - N)];
end

function [M, C, N] = arm_dyn(t, x, S)
% compute the dynamical terms of the manipulator
% M - mass matrix
% C - Corilois matrix
% N - gravity/damping forces vector

q = x(1:2);
v = x(3:4);

c1 = cos(q(1));
c2 = cos(q(2));
s2 = sin(q(2));
c12 = cos(q(1) + q(2));

% coriolis matrix
C = -S.m2*S.l1*S.lc2*s2*[v(2), v(1) + v(2);
                    -v(1), 0] + diag([.2;.2]);

% mass elements
m11 = S.m1*S.lc1^2 + S.m2*(S.l1^2 + S.lc2^2 + 2*S.l1*S.lc2*c2) + ...
      S.I1 + S.I2;

m12 = S.m2*(S.lc2^2 + S.l1*S.lc2*c2) + S.I2;

m22 = S.m2*S.lc2^2 + S.I2;

% mass matrix
M = [m11, m12;
     m12, m22];

% gravity, damping, etc...
N = [(S.m1*S.lc1 + S.m2*S.l1)*S.g*c1 + S.m2*S.lc2*S.g*c12;
      S.m2*S.lc2*S.g*c12];
end
