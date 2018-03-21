clc;
clear all; 

m = 0.27;
g = 9.81;
Ixx = 1.395e-05;
Iyy = 1.436e-05;
Izz = 2.173e-05;
Th_coef = 3.1582e-10;
To_coef = 7.9379e-12;
d = 39.73e-3;
omega_e = sqrt(m*g/4/Th_coef);

Th_coefOmega_e = (omega_e-4070.3)/0.2685;

xe = Simulink.BlockDiagram.getInitialState('try_nelin2');
ue = [omega_e omega_e omega_e omega_e];
[A,B,C,D] = linmod('try_nelin2',xe,ue);