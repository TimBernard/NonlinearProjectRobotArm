function [xs, ts] = arm_test(x0, xd)
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

% desired state
S.xd = xd;

% desired acceleration -- only applicable for trajectory tracking
S.ad = [0; 0];

T = 5; % simulation time

S.kp = 2; S.kd = 1;

[ts, xs] = ode45(@arm_ode, [0 T], x0, [], S);

% figure(12);
% plot(xs(:,1), xs(:,2), '-b')
% hold on
% 
% plot(S.xd(1),S.xd(2),'*g')
% xlabel('x')
% ylabel('y')


function u = arm_ctrl(t, x, S)
% standard computed torque law

% current
q = x(1:2);
v = x(3:4);

% desired
qd = S.xd(1:2);
vd = S.xd(3:4);

[M, C, N] = arm_dyn(t, x, S);
%u = M*(S.ad - S.kp*(q - qd) - S.kd*(v - vd)) + C*v + N;

% alternatively without coriolis/centripetal
 u = M*(S.ad - S.kp*(q - qd) - S.kd*(v - vd)) + N;



function dx = arm_ode(t, x, S)
% the ODE for the arm

v = x(3:4);

[M, C, N] = arm_dyn(t, x, S);

u = arm_ctrl(t, x, S);

dx = [v;
      inv(M)*(u - C*v - N)];



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


