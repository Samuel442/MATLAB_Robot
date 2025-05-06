function [wp_3D, wp_3D_ref] = follow_waypoints_two(wp_q, wp_q_ref, force_stop)
%FOLLOW_WAYPOINTS Summary of this function goes here
%   Detailed explanation goes here

% Convert from radian to degree
wp_q = wp_q*pi/180;
wp_q_ref = wp_q_ref*pi/180;

disp('Program started');
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)

sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID > -1)
    disp('Connected to remote API server');
else
    disp('Not connect to remote API server');
    return;
end


% Robot to Test
[errorCode, base]    = sim.simxGetObjectHandle(clientID,'UR5', sim.simx_opmode_blocking);
[errorCode, j1]      = sim.simxGetObjectHandle(clientID,'UR5_joint1', sim.simx_opmode_blocking);
[errorCode, j2]      = sim.simxGetObjectHandle(clientID,'UR5_joint2', sim.simx_opmode_blocking);
[errorCode, j3]      = sim.simxGetObjectHandle(clientID,'UR5_joint3', sim.simx_opmode_blocking);
[errorCode, j4]      = sim.simxGetObjectHandle(clientID,'UR5_joint4', sim.simx_opmode_blocking);
[errorCode, j5]      = sim.simxGetObjectHandle(clientID,'UR5_joint5', sim.simx_opmode_blocking);
[errorCode, j6]      = sim.simxGetObjectHandle(clientID,'UR5_joint6', sim.simx_opmode_blocking);
[errorCode, atuador] = sim.simxGetObjectHandle(clientID,'atuador', sim.simx_opmode_blocking);


% Robot of Reference for comparison
[errorCode, base_ref]    = sim.simxGetObjectHandle(clientID,'UR5ref', sim.simx_opmode_blocking);
[errorCode, j1_ref]      = sim.simxGetObjectHandle(clientID,'UR5_joint7', sim.simx_opmode_blocking);
[errorCode, j2_ref]      = sim.simxGetObjectHandle(clientID,'UR5_joint8', sim.simx_opmode_blocking);
[errorCode, j3_ref]      = sim.simxGetObjectHandle(clientID,'UR5_joint9', sim.simx_opmode_blocking);
[errorCode, j4_ref]      = sim.simxGetObjectHandle(clientID,'UR5_joint9', sim.simx_opmode_blocking);
[errorCode, j5_ref]      = sim.simxGetObjectHandle(clientID,'UR5_joint11', sim.simx_opmode_blocking);
[errorCode, j6_ref]      = sim.simxGetObjectHandle(clientID,'UR5_joint12', sim.simx_opmode_blocking);
[errorCode, atuador_ref] = sim.simxGetObjectHandle(clientID,'atuador0', sim.simx_opmode_blocking);

%% Robots arm control

wp_3D      = [];
wp_3D_ref  = [];

for i = 1:size(wp_q,1)
    fprintf("Waypoint: %d -->",i)
    
    % Robot to Test
    wp_i = wp_q(i,:);
    
    sim.simxSetJointTargetPosition(clientID, j1, wp_i(1), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j2, wp_i(2), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j3, wp_i(3), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j4, wp_i(4), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j5, wp_i(5), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j6, wp_i(6), sim.simx_opmode_blocking);
    
    err_ang_test = 1; 

    % Robot of Reference

    wp_ref = wp_q_ref(i,:);

    sim.simxSetJointTargetPosition(clientID, j1_ref, wp_ref(1), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j2_ref, wp_ref(2), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j3_ref, wp_ref(3), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j4_ref, wp_ref(4), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j5_ref, wp_ref(5), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j6_ref, wp_ref(6), sim.simx_opmode_blocking);

    err_ang_ref = 1; 
    

    % Wait to finish the movement

    kIter = 0;
    maxErr = 1e-1;
    maxIter = 10;
    while(norm(err_ang_test) >= maxErr && norm(err_ang_ref) >= maxErr )
        [retCode, j1_ang]   = sim.simxGetJointPosition(clientID,j1,sim.simx_opmode_blocking);
        [retCode, j2_ang]   = sim.simxGetJointPosition(clientID,j2,sim.simx_opmode_blocking);
        [retCode, j3_ang]   = sim.simxGetJointPosition(clientID,j3,sim.simx_opmode_blocking);
        [retCode, j4_ang]   = sim.simxGetJointPosition(clientID,j4,sim.simx_opmode_blocking);
        [retCode, j5_ang]   = sim.simxGetJointPosition(clientID,j5,sim.simx_opmode_blocking);
        [retCode, j6_ang]   = sim.simxGetJointPosition(clientID,j6,sim.simx_opmode_blocking);

        ang_med_1 = [j1_ang, j2_ang, j3_ang, j4_ang, j5_ang, j6_ang]; 

        err_ang_test = (ang_med_1 - wp_i)*180/pi; 

        % Reference robot
        [retCode, j1_ang_ref]   = sim.simxGetJointPosition(clientID,j1_ref,sim.simx_opmode_blocking);
        [retCode, j2_ang_ref]   = sim.simxGetJointPosition(clientID,j2_ref,sim.simx_opmode_blocking);
        [retCode, j3_ang_ref]   = sim.simxGetJointPosition(clientID,j3_ref,sim.simx_opmode_blocking);
        [retCode, j4_ang_ref]   = sim.simxGetJointPosition(clientID,j4_ref,sim.simx_opmode_blocking);
        [retCode, j5_ang_ref]   = sim.simxGetJointPosition(clientID,j5_ref,sim.simx_opmode_blocking);
        [retCode, j6_ang_ref]   = sim.simxGetJointPosition(clientID,j6_ref,sim.simx_opmode_blocking);

        ang_med_2 = [j1_ang_ref, j2_ang_ref, j3_ang_ref, j4_ang_ref, j5_ang_ref, j6_ang_ref]; 

        err_ang_ref = (ang_med_2 - wp_ref)*180/pi;
        
        kIter = kIter + 1;
        if kIter >= maxIter
            disp('Max Iteration limit')
            break
        end
    end
    
    % Robot to Test
    [retCode, position]    = sim.simxGetObjectPosition(clientID,    atuador, base, sim.simx_opmode_blocking);
    [retCode, eulerAngles] = sim.simxGetObjectOrientation(clientID, atuador, base, sim.simx_opmode_blocking);
    
    % Robot of Reference
    [retCode, position_ref]    = sim.simxGetObjectPosition(clientID,    atuador_ref, base_ref, sim.simx_opmode_blocking);
    [retCode, eulerAngles_ref] = sim.simxGetObjectOrientation(clientID, atuador_ref, base_ref, sim.simx_opmode_blocking);
    
    disp('ok')

    wp_3D = ...
            [  wp_3D;
               position,eulerAngles*180/pi];

    wp_3D_ref = ...
            [  wp_3D_ref;
               position_ref,eulerAngles_ref*180/pi];

    if force_stop
        pause
    end
end

wp_3D = double(wp_3D);
wp_3D_ref = double(wp_3D_ref);

end

