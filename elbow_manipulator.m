function f = three_link_manipulator()
% Simulation of Three Link Manipulator 

%---------- model parameters ----------%
% link dimensions 
S.length = 1;
S.height = .125;
S.width = .25;

% link masses 
S.m1 = 1;
S.m2 = 1;
S.m3 = 1;

% link lengths and mid-lengths  
S.l0 = S.length;
S.l1 = S.length;
S.l2 = S.length;
S.r0 = S.length/2;
S.r1 = S.length/2;
S.r2 = S.length/2;

% Moments of inertia 
S.Ix1 = (S.m1/12)*(S.width^2 +  S.height^2);
S.Iy1 = (S.m1/12)*(S.length^2 + S.height^2);
S.Iz1 = (S.m1/12)*(S.length^2 + S.width^2);
S.Ix2 = (S.m2/12)*(S.width^2 +  S.height^2);
S.Iy2 = (S.m2/12)*(S.length^2 + S.height^2);
S.Iz2 = (S.m2/12)*(S.length^2 + S.width^2);
S.Ix3 = (S.m3/12)*(S.width^2 +  S.height^2);
S.Iy3 = (S.m3/12)*(S.length^2 + S.height^2);
S.Iz3 = (S.m3/12)*(S.length^2 + S.width^2);

% Other parameters 
S.g = 9.8;

%---------------- Simulate Dynamics --------------------%
% Initial State 
x0 = [0 0 0 1 3 3]';

% Desired State 
S.xd = [1 1 1 0 0 0]';

% Desired Acceleration (for trajectory tracking) 
S.ad = [0 0 0]';

% Control Gains 
S.kp = 1*eye(3); 
S.kd = 2*eye(3);

T = 50; 

[ts, xs] = ode45(@three_link_ode, [0 T], x0, [], S);

% Plot executed path 
plot3(xs(:,1), xs(:,2), xs(:,3), '.');
hold on 

% Plot desired configuration 
plot3(S.xd(1),S.xd(2),S.xd(3),'*g');
xlabel('q_1')
ylabel('q_2')
zlabel('q_3')

function u = three_link_ctrl(t,x,S)
% Standard Computed Torque Law 
% u - control law 

% Current 
q = x(1:3);
v = x(4:6);

% Desired
qd = S.xd(1:3);
vd = S.xd(4:6);

[M,C,N] = three_link_dyn(t,x,S);

u = M*(S.ad - S.kd*(v-vd) - S.kp*(q-qd)) + C*v + N; 

function dx = three_link_ode(t,x,S)
% ODE for robot arm 
% dx - Dynamics

% Current 
q = x(1:3);
v = x(4:6);

[M,C,N] = three_link_dyn(t,x,S);

u = three_link_ctrl(t,x,S);
dx = [v;
      inv(M)*(-C*v -N + u)];


function [M,C,N] = three_link_dyn(t,x,S)
% Compute elements for eom of three link manipulator
% M - Mass matrix
% C - Coriolis 
% N - Gravity/friction forces

% Current
q = x(1:3);
v = x(4:6);

% Trig abbreviations 
c1 = cos(q(1)); 
c2 = cos(q(2));
c3 = cos(q(3));
c23 = cos(q(2) + q(3));
s1 = sin(q(1)); 
s2 = sin(q(2)); 
s3 = sin(q(3));
s23 = sin(q(2) + q(3));

% Mass Matrix
m11 = S.Iy2*s2^2 + S.Iy3*s23^2 + S.Iz1 + S.Iz2*c2^2 + S.Iz3*c23^2 + S.m2*S.r1^2*c2^2 + S.m3*(S.l1*c2+S.r2*c23)^2;
m12 = 0;
m13 = 0;

m21 = 0;
m22 = S.Ix2 + S.Ix3 + S.m3*S.l1^2 + S.m2*S.r1^2 + S.m3*S.r2^2 + 2*S.m3*S.l1*S.r2*c3;
m23 = S.Ix3 + S.m3*S.r2^2 + S.m3*S.l1*S.r2*c3;

m31 = 0;
m32 = S.Ix3 + S.m3*S.r2^2 + S.m3*S.l1*S.r2*c3;
m33 = S.Ix3 + S.m3*S.r2^2;

M = [m11, m12, m13; 
     m21, m22, m23; 
     m31, m32, m33];

% Coriolis Matrix 
gamma112 = (S.Iy2 - S.Iz2 - S.m2*S.r1^2)*c2*s2 + (S.Iy3 - S.Iz3)*c23*s23 - S.m3*(S.l1*c2 + S.r2*c23)*(S.l1*s2 + S.r2*s23);
gamma113 = (S.Iy3-S.Iz3)*c23*s23 - S.m3*S.r2*s23*(S.l1*c2 + S.r2*c23);
gamma121 = (S.Iy2 - S.Iz2 - S.m2*S.r1^2)*c2*s2 + (S.Iy3 - S.Iz3)*c23*s23 - S.m3*(S.l1*c2 + S.r2*c23)*(S.l1*s2 + S.r2*s23);
gamma131 = (S.Iy3-S.Iz3)*c23*s23 - S.m3*S.r2*s23*(S.l1*c2 + S.r2*c23);

gamma211 = (S.Iz2 - S.Iy2 + S.m2*S.r1^2)*c2*s2 + (S.Iz3 - S.Iy3)*c23*s23 + S.m3*(S.l1*c2 + S.r2*c23)*(S.l1*s2 + S.r2*s23);
gamma223 = -S.l1*S.m3*S.r2*s3;
gamma232 = -S.l1*S.m3*S.r2*s3;
gamma233 = -S.l1*S.m3*S.r2*s3;

gamma311 = (S.Iz3 - S.Iy3)*c23*s23 + S.m3*S.r2*s23*(S.l1*c2 + S.r2*c23);
gamma322 = S.l1*S.m3*S.r2*s3;

% Coriolis Matrix Elements 
c11 = gamma112*v(2) + gamma113*v(3);
c12 = gamma121*v(1);
c13 = gamma131*v(1);

c21 = gamma211*v(1);
c22 = gamma223*v(3);
c23 = gamma232*v(2) + gamma233*v(3);

c31 = gamma311*v(1);
c32 = gamma322*v(2);
c33 = 0; 

C = [c11, c12, c13; 
     c21, c22, c23;
     c31, c32, c33];

% Other external forces 
N = [0;
     -(S.m2*S.g*S.r1 + S.m3*S.g*S.l1)*c2 - S.m3*S.r2*c23;
     -S.m3*S.g*S.r2*c23];
