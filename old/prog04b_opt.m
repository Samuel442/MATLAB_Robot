clear, clc, close all

load("UR5_T_static.mat");
load("UR5_WayPoints.mat")

% global T_static_total pos_desired alpha_desired



tam = size(WP_q,1);

% 
% pos_desired    = pos;
% alpha_desired  = ang;
% T_static_total = T_static;

% Axis around each angle of q rotates
u_j_static = [ 0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1]; 

WP_3D_2 = [];

for i=1:tam
    q = WP_q(i,:);

    % Compute UTM for each joitn 
    T_juntas = calc_T_utm_dynamic(T_static(:,:,1:6), q, u_j_static);
    
    % Add the effect of the actuator
    T_final = T_juntas*T_static(:,:,7);
    
    % Extract the position and orientantion of the actator in robot frame
    [position, eulerAngles] = calc_T_utm_extract_pos_ang(T_final);

    WP_3D_2 = [  WP_3D_2;
                 position', eulerAngles'*180/pi  ];
end


WP_3D
WP_3D_2


for i=1:tam
    disp('  ')
    disp(i)
    disp(num2str([WP_3D(i,:); WP_3D_2(i,:)]))
end