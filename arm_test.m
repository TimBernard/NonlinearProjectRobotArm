function [xs, ts] = arm_test(x0, xf, S)
% simulation of a two-link manipulator 
% desired state
S.xd = xf;
q0 = x0(1:2);
qf = xf(1:2);
dq0 = [x0(3);x0(4)];  % desired starting velocity
dqf = [xf(3);xf(4)]; % desired end velocity

T = 5; % simulation time

% compute path coefficients
A = poly3_coeff(q0, dq0, qf, dqf, T);
S.A = A;

% desired path
qd = A*poly3([0:.01:T]);

[ts, xs] = ode45(@arm_ode, [0 T], x0, [], S);

figure(5);
hold on
plot(xs(:,1), xs(:,2), '-b')

plot(qd(1,:),qd(2,:),'--r')
xlabel('q1')
ylabel('q2')
legend('Trajectory','Desired')
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
q = x(1:2);
dq = x(3:4);

[M, C, N] = arm_dyn(t, x, S);
b = C*dq + N;

% virtual input
v = d2qd - S.kp*(q - qd) - S.kd*(dq - dqd);

% inputs
u = M*v + b;
end

function dx = arm_ode(t, x, S)
% the ODE for the arm

dq = x(3:4);

[M, C, N] = arm_dyn(t, x, S);
b = C*dq + N;

u = arm_ctrl(t, x, S);

dx = [dq;
      inv(M)*(u-b)]
      
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

