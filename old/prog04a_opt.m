clear, clc, close all

load("UR5_T_static.mat");
load("UR5_WayPoints.mat")

% Algorithm

tam = size(WP_3D,1);

q_est = [];
q0 = zeros(6,1);

for i = 1:tam
    pos = WP_3D(i,1:3)';
    ang = round(WP_3D(i,4:6),5)';

    q0 = WP_q(i,:);

    q_star = ur5_IK_UTM(pos,ang,T_static,q0);
    % q0 = q_star;

    q_est = [ q_est;
              q_star];

    % disp(num2str(q_star*180/pi))
end


%%
% disp(num2str(WP_q))
% disp(num2str(round(q_est*180/pi,2)))


for i=1:size(WP_q,1)
    disp('  ')
    disp(i)
    disp(num2str([WP_q(i,:); round(q_est(i,:),2)]))
end

%%
% [WP_3D_est] = follow_waypoints(q_est)

[WP_3D_2, WP_3D_ref] = follow_waypoints_two(q_est, WP_q)

