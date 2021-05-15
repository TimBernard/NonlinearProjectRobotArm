function [xs, ts, error] = arm_testLR(x0, xf, S)
% simulation of a two-link manipulator 
% desired state
S.xd = xf;
q0 = x0(1:3);
qf = xf(1:3);
dq0 = x0(4:6); % desired starting velocity
dqf = xf(4:6); % desired end velocity

T = 0.1; % simulation time

% compute path coefficients
A = poly3_coeff(q0, dq0, qf, dqf, T);
S.A = A;

[ts, xs] = ode45(@arm_ode, [0 T], x0, [], S);

% desired path
qd = A*poly3(ts');
error = [];
for i = 1:length(xs)
    error = [error, norm(qd(:,i) - xs(i,1:3)')];
end

figure(7);
hold on
plot3(xs(:,1), xs(:,2), xs(:,3), '-b')

plot3(qd(1,:),qd(2,:),qd(3,:),'--r')
xlabel('q1')
ylabel('q2')
zlabel('q3')
legend('Trajectory','Desired')
%axis equal
end

%% Trajectory Generation
function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
A = Y*inv(L);
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

%% Control Law
function u = arm_ctrl(t, x, S)
% desired
qd = S.A*poly3(t);
dqd = S.A*dpoly3(t);
d2qd = S.A*d2poly3(t);

% current
q = x(1:3);
dq = x(4:6);

[M, C, N] = arm_dyn(t, x, S);
b = C + N;

% virtual input
v = d2qd - S.kp*(q - qd) - S.kd*(dq - dqd);

% inputs
u = M*v + b;
end

function dx = arm_ode(t, x, S)
% the ODE for the arm
dq = x(4:6);

k0 = 0.30;
nu = k0 * abs(dq(3));

[M, C, N] = arm_dyn(t, x, S);
b = C + N;

u = arm_ctrl(t, x, S);

if norm(dq) > 1
    v = -nu*(dq/norm(dq));
else
    v = -(nu^2)*(dq)/1;
end

delta = [0;0;(2*rand()-1)*k0*abs(dq(3))];
%delta = (2*rand()-1)*k0*q;

dx = [dq;
      inv(M)*(u + v + delta - b)];
      
end

function [M, C, N] = arm_dyn(t, x, S)
% compute the dynamical terms of the manipulator
% M - mass matrix
% C - Corilois matrix
% N - gravity/damping forces vector

q = x(1:3);
v = x(4:6);

c1 = cos(q(1));
c2 = cos(q(2));
c3 = cos(q(3));
s1 = sin(q(1));
s2 = sin(q(2));
s3 = sin(q(3));
c12 = cos(q(1) + q(2));
c23 = cos(q(2) + q(3));
c123 = cos(q(1) + q(2) + q(3));
s23 = sin(q(2) + q(3));

% coriolis elements
b1 = -S.m2*S.l1*S.lc2*(2*v(1) + v(2))*s2*v(2) - ...
     S.m3*(S.l1*S.l2*(2*v(1) + v(2))*s2 + S.l1*S.lc3*(2*v(1) + v(2) + v(3))*s23 + S.l2*S.lc3*(2*v(1) + 2*v(2) + v(3))*s3);

b2 = -S.m2*(S.l1*S.lc2*(v(1)^2 + v(1)*v(2))*s2 + S.l1*S.lc2*v(1)*s2*v(2)) ...
     - S.m3*(S.l1*S.l2*(v(1)^2 + v(1)*v(2)) + S.l1*S.lc3*(v(1)^2 + ...
     v(1)*v(2) + v(1)*v(3))*s23 + S.l2*S.lc3*(v(1)^2 + 2*v(1)*v(2) + ...
     v(1)*v(3) + v(2)^2 + v(2)*v(3))*s3 + S.l1*S.l2*v(1)*s2*v(2) + ...
     S.l1*S.lc3*v(1)*v(2)*s23 + S.l2*S.lc3*(2*v(1) + 2*v(2) + v(3))*s3);

b3 = -S.m3*(S.l1*S.lc3*(v(1)^2 + v(1)*v(2) + v(1)*v(3))*s23 + ...
     S.l2*S.lc3*(v(1)^2 + 2*v(1)*v(2) + v(1)*v(3) + v(2)^2 + ...
     v(2)*v(3))*s3 + S.l1*S.lc3*v(1)*v(3)*s23 + S.l2*S.lc3*(v(1) + v(2))*s3*v(3));

% coriolis matrix
C = [b1; 
     b2; 
     b3];

% mass elements
m11 = S.m1*S.lc1^2 + S.m2*(S.l1^2 + S.lc2^2 + 2*S.l1*S.lc2*c2) + ...
      S.m3*(S.l1^2 + S.l2^2 + S.lc3^2 + 2*S.l1*S.l2*c2 + ...
      2*S.l1*S.lc3*c23 + 2*S.l2*S.lc3*c3) + S.I1 + S.I2 + S.I3;

m12 = S.m2*(S.lc2^2 + S.l1*S.lc2*c2) + S.m3*(S.l2^2 + S.lc3^2 + ...
      S.l1*S.l2*c2 + S.l1*S.lc3*c23 + 2*S.l2*S.lc3*c3) + S.I2 + S.I3;
  
m13 = S.m3*(S.lc3^2 + S.l1*S.lc3*c23 + S.l2*S.lc3*c3) + S.I3;

m22 = S.m2*S.lc2^2 + S.m3*(S.l2^2 + S.lc3^2 + 2*S.l2*S.lc3*c3) + S.I2 + S.I3;

m23 = S.m3*(S.lc3^2 + S.l2*S.lc3*c3) + S.I3;

m33 = S.m3*S.lc3^2 + S.I3;

% mass matrix
M = [m11, m12, m13;
     m12, m22, m23;
     m13, m23, m33];

% gravity, damping terms
g1 = c1*(S.m1*S.lc1 + S.m2*S.l1 + S.m3*S.l1) + c12*(S.m2*S.lc2 + ...
     S.m3*S.l2) + c123*S.m3*S.lc3;

g2 = c12*(S.m2*S.lc2 + S.m3*S.l2) + c123*S.m3*S.lc3;

g3 = c123*S.m3*S.lc3;

% gravity, damping, etc...
N = S.g *[g1;
          g2;
          g3];
end

