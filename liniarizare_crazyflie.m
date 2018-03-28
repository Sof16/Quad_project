clc;
clear all; 

format long;

m = 0.027;
mt = 0.033;
g = 9.81;
Ixx = 1.395e-05;
Iyy = 1.436e-05;
Izz = 2.173e-05;
Th_coef = 3.1582e-10;
To_coef = 7.9379e-12;
d = 39.73e-3;
omega_e = sqrt(mt*g/4/Th_coef);

Th_coefOmega_e = (omega_e-4070.3)/0.2685;

xe = Simulink.BlockDiagram.getInitialState('try_nelin2');
ue = [omega_e omega_e omega_e omega_e];
[A,B,C,D] = linmod('try_nelin2',xe,ue);
% 
% Q = diag([2000; 2000; 4000; 4000; 4000; 4000; 20; 20; 10; 10; 10; 10]);
% R = 0.00003*diag([1;1;1;1]);
% 
% K = lqr(A,B,Q,R);
% 
% A = A-B*K;