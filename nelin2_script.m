clc;
clear all;

syms x y z yaw pitch roll u v w r p q m g Ixx Izz Iyy Ct Cd d u1 u2 u3 u4

% m = 0.27;
% g = 9.81;
% Ixx = 1.395e-05;
% Iyy = 1.436e-05;
% Izz = 2.173e-05;
% Th_coef = 3.1582*10e-10;
% To_coef = 7.9379*10e-12;
% d = 39.73*10e-3;
% omega_e = 16073;

    %dx = [x y z yaw pitch roll x_rate y_rate z_rate yaw_rate pitch_rate
                %roll_rate]
    dx = zeros(12,1);
    
    Rot_InertialToBody = [cos(pitch)*cos(yaw) cos(pitch)*sin(yaw) -sin(pitch); ...
    sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw) sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw) sin(roll)*cos(pitch); ...
    cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw) cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw)  cos(roll)*cos(yaw)];
    
    dx1 = Rot_InertialToBody(1:3,1)'*[u; v; w];
    dx2 = Rot_InertialToBody(1:3,2)'*[u; v; w];
    dx3 = Rot_InertialToBody(1:3,3)'*[u; v; w];
    
    Rot_angles = [1 sin(roll)*tan(pitch) cos(roll)*tan(pitch); ...
        0 cos(roll) -sin(roll); ...
        0 sin(roll)/cos(pitch) cos(roll)/cos(pitch)];
    
    dx4 = Rot_angles(1,1:3)*[p; q; r];
    dx5 = Rot_angles(2,1:3)*[p; q; r];
    dx6 = Rot_angles(3,1:3)*[p; q; r];
    
    % Ct = thrust coeff and Cd = aerodinamical drag
    Fz = Ct*(u1^2 + u2^2 + u3^2 + u4^2);
    
    cross_prod_rate = cross([p; q; r], [u; v; r]);
    second_termen = Rot_InertialToBody*[0;0;g];
    
    dx7 = - second_termen(1) - cross_prod_rate(1); 
    dx8 = - second_termen(2) - cross_prod_rate(2); 
    dx9 = Fz/m - second_termen(3) - cross_prod_rate(3); 
    
    Inertia_matrix = diag([Ixx Iyy Izz]);
    Mx = d*Ct/sqrt(2)*(-u1^2 - u2^2 + u3^2 + u4^2);
    My = d*Ct/sqrt(2)*(-u1^2 + u2^2 + u3^2 - u4^2);
    Mz = Cd*(-u1^2 + u2^2 - u3^2 + u4^2);
    
    Last_states = inv(Inertia_matrix)*([Mx; My; Mz] - cross([p; q; r], Inertia_matrix*[p; q; r]));
    dx10 = Last_states(3);
    dx11 = Last_states(2);
    dx12 = Last_states(1);
    
