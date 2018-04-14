clc;
clear all; 

format long;

m = 0.027;
mt = 0.03327;
g = 9.81;
Ixx = 1.395e-05;
Iyy = Ixx; %1.436e-05;
Izz = 2.173e-05;
Th_coef = 3.1582e-10;
To_coef = 7.9379e-12;
d = 39.73e-3;
omega_e = 16073;

Th_coefOmega_e = (omega_e-4070.3)/0.2685;

xe = Simulink.BlockDiagram.getInitialState('try_nelin2');
% ue = [omega_e omega_e omega_e omega_e];
pwm = 44705;
ue = [pwm pwm pwm pwm];
[A1,B1,C1,D1] = linmod('try_nelin2',xe,ue);

B1 = round(B1,6);

% Q = diag([2000; 2000; 4000; 4000; 4000; 4000; 20; 20; 10; 10; 10; 10]);
% R = 0.00003*diag([1;1;1;1]);
% 
% K = lqr(A,B,Q,R);
% 
% A = A-B*K;