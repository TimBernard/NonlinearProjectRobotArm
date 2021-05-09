close all

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

S.kp = 2*10^2; S.kd = 1*10^2;

q0 = Q(:,path_indx(1))
qf = Q(:,path_indx(2))

arm_test([q0;0;0;0], [qf;0;0;0], S);