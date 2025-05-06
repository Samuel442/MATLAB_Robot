function [J] = ur5_J_a(q)
% q

q = q * pi/180;

global T_static_total pos_obj ang_obj

T_static = T_static_total;

% Axis around each angle of q rotates
u_i_static = [ 0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1]; 


% Compute UTM for each joitn 
T_juntas = calc_T_dynamic(T_static(:,:,1:6), q, u_i_static);

% Add the effect of the actuator
T_est = T_juntas*T_static(:,:,7);

% Extract the position and orientantion of the actator in robot frame
[u_est, ang_est, pos_est] = calc_T_extract_u_angle(T_est);

ang_est = ang_est*u_est; % Vector representation of Euler Angle/Axis

err_pos =  1000*(pos_est  -  pos_obj).^2;
err_ang =       (ang_est  -  ang_obj).^2;

% err_vec = [ err_pos;
%             err_ang  ];


J = 1000*norm(err_pos) + 10*norm(err_ang) ;
end

