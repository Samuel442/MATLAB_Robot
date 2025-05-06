clear, clc, close all

disp('Programa iniciado');
sim = remApi('remoteApi'); % usando o arquivo de protótipo (remoteApiProto.m)

sim.simxFinish(-1); % por precaução, fecha todas as conexões abertas
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID > -1)
    disp('Conectado ao servidor API remota');
else
    disp('Não conectado ao servidor API remota');
    return;
end


[errorCode, base]    = sim.simxGetObjectHandle(clientID,'robotRef'  , sim.simx_opmode_blocking);
[errorCode, j1]      = sim.simxGetObjectHandle(clientID,'UR5_joint1', sim.simx_opmode_blocking);
[errorCode, j2]      = sim.simxGetObjectHandle(clientID,'UR5_joint2', sim.simx_opmode_blocking);
[errorCode, j3]      = sim.simxGetObjectHandle(clientID,'UR5_joint3', sim.simx_opmode_blocking);
[errorCode, j4]      = sim.simxGetObjectHandle(clientID,'UR5_joint4', sim.simx_opmode_blocking);
[errorCode, j5]      = sim.simxGetObjectHandle(clientID,'UR5_joint5', sim.simx_opmode_blocking);
[errorCode, j6]      = sim.simxGetObjectHandle(clientID,'UR5_joint6', sim.simx_opmode_blocking);
[errorCode, atuador] = sim.simxGetObjectHandle(clientID,'atuador'   , sim.simx_opmode_blocking);


%%  Avalia o cálculo do teorico e prático da matriz T 

load("UR5_T_static.mat");

WP_q      =    [    0,   0,   0,   0,   0,   0;
                   10,  15, -10, -15, -20,  10;
                   80,  30,  45,  50,  20, 100;
                 -100, -50, -30,   0, 100,  -5;
                  -10,  30,  35,  25,  15,  50;
                  150,  75,  20,  10,  -5,  15;
                    0,   0,   0,   0,   0,   0];

% WP_q      =    [    0,   0,   0,   0,   0,   0;
%                     90,   0,   0,   0,   0,   0;
%                     0,   90,   0,   0,   0,   0;
%                     0,   0,   90,   0,   0,   0;
%                     0,   0,   0,   90,   0,   0;
%                     0,   0,   0,   0,   90,   0;
%                     0,   0,   0,   0,   0,   90;
%                     0,   0,   0,   0,   0,   0;];

% Eixo em torno do qual cada ângulo de q rotaciona
u_i_static = [ 0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1; 
               0, 0, 1]; 

tam = size(WP_q,1);

WP_3D_sim = [];
WP_3D_calc = [];

for i=1:tam
    disp(i)
    % % % Definindo o espaço de configuração
    qi = WP_q(i,:);
    disp(['Angulos:   ', num2str(qi)])
    qi = qi*pi/180;
    
    sim.simxSetJointTargetPosition( clientID, j1, qi(1), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j2, qi(2), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j3, qi(3), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j4, qi(4), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j5, qi(5), sim.simx_opmode_blocking);
    sim.simxSetJointTargetPosition(clientID, j6, qi(6), sim.simx_opmode_blocking);
    
    err_ang = 1; 
    ang_med = 0; 

    maxIter = 25;
    kIter = 0;
    while(norm(err_ang) >= 1e-1)
        [returnCode, j1_ang]  = sim.simxGetJointPosition(clientID,j1,sim.simx_opmode_blocking);
        [returnCode, j2_ang]  = sim.simxGetJointPosition(clientID,j2,sim.simx_opmode_blocking);
        [returnCode, j3_ang]  = sim.simxGetJointPosition(clientID,j3,sim.simx_opmode_blocking);
        [returnCode, j4_ang]  = sim.simxGetJointPosition(clientID,j4,sim.simx_opmode_blocking);
        [returnCode, j5_ang]  = sim.simxGetJointPosition(clientID,j5,sim.simx_opmode_blocking);
        [returnCode, j6_ang]  = sim.simxGetJointPosition(clientID,j6,sim.simx_opmode_blocking);

        ang_med = [j1_ang, j2_ang, j3_ang, j4_ang, j5_ang, j6_ang]; 

        err_ang = (ang_med - qi)*180/pi; 

        kIter = kIter + 1;
        if kIter >= maxIter
            disp('Max Iteration limit')
            break
        end
    end

    % % % Obtem a posicao e atitude do atuador em relacao a base

    [returnCode, sim_pos]    =    sim.simxGetObjectPosition(clientID, atuador, base, sim.simx_opmode_blocking);
    [returnCode, sim_ang]    = sim.simxGetObjectOrientation(clientID, atuador, base, sim.simx_opmode_blocking);
    sim_pos = double(sim_pos);
    sim_ang = double(sim_ang);
    
    disp(['Sim Posição:     ', num2str(sim_pos,6)])
    disp(['Sim Angulos de Euler: ', num2str(sim_ang*180/pi)])
    

    % % % Compute the theoretical values 

    % Compute UTM for each joitn 
    T_juntas = calc_T_dynamic(T_static(:,:,1:6), qi, u_i_static);
    
    % Add the effect of the actuator
    T_final = T_juntas*T_static(:,:,7)
    
    % Extract the position and orientantion of the actator in robot frame

    [position, eulerAngles] = calc_T_extract_pos_ang(T_final);
    [u,angle,p] = calc_T_extract_u_angle(T_final);

    T2 = calc_T_u_angle(angle,u,p)
    % disp(['Calc Position:     ', num2str(position',6)])
    % disp(['Calc Euler Angles: ', num2str(eulerAngles'*180/pi)])
    % disp(' ')

    % % % Save data

    WP_3D_sim = [  WP_3D_sim;sim_pos, sim_ang*180/pi  ];
    % 
    WP_3D_calc = [  WP_3D_calc; position', eulerAngles'*180/pi  ];
    

    pause
end

%% 

WP_3D_sim
WP_3D_calc


% % for i=1:tam
% %     disp('  ')
% %     disp(i)
% %     disp(num2str([WP_3D(i,:); WP_3D(i,:)]))
% % end