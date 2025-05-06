clear, clc, close all

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

%% Algorithm

WP_q      =    [   0,   0,   0,   0,   0,   0;
                  10,  15, -10, -15, -20,  10;
                  80,  30,  45,  50,  20, 100;
                -100, -50, -30,   0, 100,  -5;
                 -10,  30,  35,  25,  15,  50;
                 150,  75,  20,  10,  -5,  15;
                   0,   0,   0,   0,   0,   0];


[errorCode, base]    = sim.simxGetObjectHandle(clientID,'UR5', sim.simx_opmode_blocking);
[errorCode, j1]      = sim.simxGetObjectHandle(clientID,'UR5_joint1', sim.simx_opmode_blocking);
[errorCode, j2]      = sim.simxGetObjectHandle(clientID,'UR5_joint2', sim.simx_opmode_blocking);
[errorCode, j3]      = sim.simxGetObjectHandle(clientID,'UR5_joint3', sim.simx_opmode_blocking);
[errorCode, j4]      = sim.simxGetObjectHandle(clientID,'UR5_joint4', sim.simx_opmode_blocking);
[errorCode, j5]      = sim.simxGetObjectHandle(clientID,'UR5_joint5', sim.simx_opmode_blocking);
[errorCode, j6]      = sim.simxGetObjectHandle(clientID,'UR5_joint6', sim.simx_opmode_blocking);
[errorCode, atuador] = sim.simxGetObjectHandle(clientID,'atuador', sim.simx_opmode_blocking);


WP_3D = [];

for k = 1:size(WP_q,1)
    disp(k)
    
    wp = WP_q(k,:)*pi/180;
    
    sim.simxSetJointTargetPosition(clientID, j1, wp(1), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j2, wp(2), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j3, wp(3), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j4, wp(4), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j5, wp(5), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j6, wp(6), sim.simx_opmode_blocking);
    
    err_ang = 1; 
    ang_med = 0; 

    maxIter = 25;
    kIter = 0;
    while(norm(err_ang) >= 1e-1)
        [returnCode, j1_ang]   = sim.simxGetJointPosition(clientID,j1,sim.simx_opmode_blocking);
        [returnCode, j2_ang]   = sim.simxGetJointPosition(clientID,j2,sim.simx_opmode_blocking);
        [returnCode, j3_ang]   = sim.simxGetJointPosition(clientID,j3,sim.simx_opmode_blocking);
        [returnCode, j4_ang]   = sim.simxGetJointPosition(clientID,j4,sim.simx_opmode_blocking);
        [returnCode, j5_ang]   = sim.simxGetJointPosition(clientID,j5,sim.simx_opmode_blocking);
        [returnCode, j6_ang]   = sim.simxGetJointPosition(clientID,j6,sim.simx_opmode_blocking);

        ang_med = [j1_ang, j2_ang, j3_ang, j4_ang, j5_ang, j6_ang]; 

        err_ang = (ang_med - wp)*180/pi; 

        kIter = kIter + 1;
        if kIter >= maxIter
            disp('Max Iteration limit')
            break
        end
    end
    
    
    disp(['Err_mod: ', num2str(norm(err_ang))])
    disp(['err_ang: ', num2str(err_ang)])
    disp(['angles: ', num2str(ang_med*180/pi)])
    
    % [returnCode, position]   = sim.simxGetObjectPosition(clientID,atuador,-1,sim.simx_opmode_blocking);
    % [returnCode, eulerAngles] = sim.simxGetObjectOrientation(clientID, atuador, -1, sim.simx_opmode_blocking);
    [returnCode, position]    =    sim.simxGetObjectPosition(clientID, atuador, base, sim.simx_opmode_blocking);
    [returnCode, eulerAngles] = sim.simxGetObjectOrientation(clientID, atuador, base, sim.simx_opmode_blocking);
    disp(['Position:     ', num2str(position,6)])
    disp(['Euler Angles: ', num2str(eulerAngles*180/pi)])
    disp(' ')
    % pause(0.5)

    WP_3D = [  WP_3D;
               position,eulerAngles*180/pi];

    pause       
end

WP_3D = double(WP_3D)

save('UR5_WayPoints.mat',"WP_q","WP_3D")

