function [q_star] = ur5_IK_c_partswarm(T_ref,T_static, q0)
%UR5_IK_UTM Summary of this function goes here
%   Detailed explanation goes here

% Load the T_static

[u_ref, ang_ref, p_ref] = calc_T_extract_u_angle(T_ref);

global T_static_total pos_obj ang_obj

T_static_total = T_static;
pos_obj = p_ref;
ang_obj = ang_ref*u_ref;

% Optimization
% upb = pi*ones(6,1);
% lob = -1*upb;

upb = 179*ones(6,1);
lob = -1*upb;


%%  Nonlinear Optmization

q_star = particleswarm(@ur5_J_c, 6, lob, upb);

end
