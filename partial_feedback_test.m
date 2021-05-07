function f = arm_test()
clear variables; clc; close all; 
% EN530.678 HW#3 supplementary 
% simulation of a two-link manipulator 
%
% M. Kobilarov

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

% initial state with high-velocity
x0 = [0; 0; 2; 5] ;

% desired state
xf = [pi; 3*pi/2; 0; 0];

% Set initial and final velocity 
S.vel = 1;

T = 10; % simulation time

% Gains 
S.kp = 2; S.kd = 1;

% boundary conditions in flat output space 
y0 = arm_h(x0);
yf = arm_h(xf);
dy0 = [x0(3),x0(4)]';  % desired starting velocity
dyf = [xf(3),xf(4)]'; % desired end velocity

% compute path coefficients
A = poly3_coeff(y0, dy0, yf, dyf, T);
S.A = A;

figure(1)
hold on
% plot desired path
X_ = A*poly3([0:.01:T]);
plot(X_(1,:), X_(2,:), '-r');

x0 = x0  + [.15;.150;0;0];

[ts, xs] = ode45(@arm_ode, [0 T], x0, [], S);

% plot executed path
plot(xs(:,1), xs(:,2), '-b')

% plot goal configuration 
plot(xf(1),xf(2),'*g')
xlabel('q_1')
ylabel('q_2')
title('2D Manipulator trajectory routine')
hold off 

figure(2)
hold on 

% Desired q2
plot(linspace(0,10,size(X_(2,:),2)),X_(2,:),'-r');

% Exectued q2 
plot(ts,xs(:,2),'-b');

% plot goal q2 joint value 
plot(10,xf(2),'*g');

xlabel('time');
ylabel('q_2'); 
title('q_2 path');
hold off
    
figure(3)
hold on 

% Desired q1
plot(linspace(0,10,size(X_(1,:),2)),X_(1,:),'-r');

% Exectued q2 
plot(ts,xs(:,1),'-b');

% plot goal q2 joint value 
plot(10,xf(1),'*g');

xlabel('time');
ylabel('q_1'); 
title('q_1 path');
hold off

function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
A = Y*inv(L);

function y = arm_h(x)
% output function
y = x(1:2);

function f = poly3(t)
f = [t.^3; t.^2; t; ones(size(t))];

function f = dpoly3(t)
f = [3*t.^2; 2*t; ones(size(t)); zeros(size(t))];

function f = d2poly3(t)
f = [6*t; 2; zeros(size(t)); zeros(size(t))];

function u = arm_ctrl(t, x, S)
% Modified for collacted input/output Linearization 
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

% Set bias equal to Coriolis + centrifugal forces + gravity, damping etc.
b = C*v + N; 

% Definitions 
m22_ = M(2,2) -M(2,1)*inv(M(1,1))*M(1,2);
b2_ = b(2) - M(2,1)*inv(M(1,1))*b(1);

% Virtual Input 
virtual = ad(2) -S.kd*(v(2)-vd(2)) -S.kp*(q(2)-qd(2));

% Control to stabilize to q2 trajectory 
u = m22_*virtual + b2_; 

function dx = arm_ode(t, x, S)
% the ODE for the arm
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% desired trajectory and controls
qd = yd;
vd = dyd;
ad = d2yd;

q = x(1:2); 
v = x(3:4);

[M, C, N] = arm_dyn(t, x, S);
% Set bias equal to Coriolis + centrifugal forces + gravity, damping etc.
b = C*v + N; 

% Definitions 
m22_ = M(2,2) -M(2,1)*inv(M(1,1))*M(1,2);
b2_ = b(2) - M(2,1)*inv(M(1,1))*b(1);

u = arm_ctrl(t, x, S);

dx = [v;
       -inv(M(1,1))*(M(1,2)*(ad(2) -S.kd*(v(2)-vd(2)) -S.kp*(q(2)-qd(2)))+b(1));
        inv(m22_)*(u-b2_)];    


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