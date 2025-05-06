clear, clc, close all

disp('Programa iniciado');
sim = remApi('remoteApi'); %usando o arquivo prototipo (remoteApiProto.m)

sim.simxFinish(-1); % por precaucao, feche todas as conexoes abertas
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID > -1)
    disp('Conectado ao servidor da API remota');
else
    disp('Nao conectado ao servidor da API remota');
    return;
end

%% Conecção

[errorCode, ref]     = sim.simxGetObjectHandle(clientID,'robotRef', sim.simx_opmode_blocking);
[errorCode, j1]      = sim.simxGetObjectHandle(clientID,'UR5_joint1', sim.simx_opmode_blocking);
[errorCode, j2]      = sim.simxGetObjectHandle(clientID,'UR5_joint2', sim.simx_opmode_blocking);
[errorCode, j3]      = sim.simxGetObjectHandle(clientID,'UR5_joint3', sim.simx_opmode_blocking);
[errorCode, j4]      = sim.simxGetObjectHandle(clientID,'UR5_joint4', sim.simx_opmode_blocking);
[errorCode, j5]      = sim.simxGetObjectHandle(clientID,'UR5_joint5', sim.simx_opmode_blocking);
[errorCode, j6]      = sim.simxGetObjectHandle(clientID,'UR5_joint6', sim.simx_opmode_blocking);
[errorCode, atuador] = sim.simxGetObjectHandle(clientID,'atuador', sim.simx_opmode_blocking);



[returnCode, j1_ang]   = sim.simxGetJointPosition(clientID,j1,sim.simx_opmode_blocking);
[returnCode, j2_ang]   = sim.simxGetJointPosition(clientID,j2,sim.simx_opmode_blocking);
[returnCode, j3_ang]   = sim.simxGetJointPosition(clientID,j3,sim.simx_opmode_blocking);
[returnCode, j4_ang]   = sim.simxGetJointPosition(clientID,j4,sim.simx_opmode_blocking);
[returnCode, j5_ang]   = sim.simxGetJointPosition(clientID,j5,sim.simx_opmode_blocking);
[returnCode, j6_ang]   = sim.simxGetJointPosition(clientID,j6,sim.simx_opmode_blocking);
[returnCode, j7_ang]   = sim.simxGetJointPosition(clientID,j6,sim.simx_opmode_blocking); % verificar


%% Parametros

ang_med = [j1_ang, j2_ang, j3_ang, j4_ang, j5_ang, j6_ang, j7_ang];


disp(' ')
disp(['Angulo das juntas: ', num2str(ang_med*180/pi)]) % converte o ângulo radianos em graus e strig
disp(' ')


[returnCode, j1_pos]  =    sim.simxGetObjectPosition    (clientID,  j1, ref, sim.simx_opmode_blocking);
[returnCode, j1_eul]  = sim.simxGetObjectOrientation(clientID,  j1, ref, sim.simx_opmode_blocking);
disp('1')
disp(['Posicao: '    , num2str(j1_pos)])
disp(['Angulos de Euler: ', num2str(j1_eul*180/pi)])
disp(' ')


[returnCode, j2_pos]  =    sim.simxGetObjectPosition(clientID,  j2, j1, sim.simx_opmode_blocking);
[returnCode, j2_eul]  = sim.simxGetObjectOrientation(clientID,  j2, j1, sim.simx_opmode_blocking);
disp('2')
disp(['Posicao: '    , num2str(j2_pos)])
disp(['Angulos de Euler: ', num2str(j2_eul*180/pi)])
disp(' ')


[returnCode, j3_pos]  =    sim.simxGetObjectPosition(clientID,  j3, j2, sim.simx_opmode_blocking);
[returnCode, j3_eul]  = sim.simxGetObjectOrientation(clientID,  j3, j2, sim.simx_opmode_blocking);
disp('3')
disp(['Posicao: '    , num2str(j3_pos)])
disp(['Angulos de Euler: ', num2str(j3_eul*180/pi)])
disp(' ')


[returnCode, j4_pos]  =    sim.simxGetObjectPosition(clientID,  j4, j3, sim.simx_opmode_blocking);
[returnCode, j4_eul]  = sim.simxGetObjectOrientation(clientID,  j4, j3, sim.simx_opmode_blocking);
disp('4')
disp(['Posicao: '    , num2str(j4_pos)])
disp(['Angulos Euler: ', num2str(j4_eul*180/pi)])
disp(' ')

[returnCode, j5_pos]  =    sim.simxGetObjectPosition(clientID,  j5, j4, sim.simx_opmode_blocking);
[returnCode, j5_eul]  = sim.simxGetObjectOrientation(clientID,  j5, j4, sim.simx_opmode_blocking);
disp('5')
disp(['Posicao: '    , num2str(j5_pos)])
disp(['Angulos Euler: ', num2str(j5_eul*180/pi)])
disp(' ')


[returnCode, j6_pos]  =    sim.simxGetObjectPosition(clientID,  j6, j5, sim.simx_opmode_blocking);
[returnCode, j6_eul]  = sim.simxGetObjectOrientation(clientID,  j6, j5, sim.simx_opmode_blocking);
disp('6')
disp(['Posicao: '    , num2str(j6_pos)])
disp(['Angulos Euler: ', num2str(j6_eul*180/pi)])
disp(' ')


[returnCode, j7_pos]  =    sim.simxGetObjectPosition(clientID, atuador, j6, sim.simx_opmode_blocking);
[returnCode, j7_eul]  = sim.simxGetObjectOrientation(clientID, atuador, j6, sim.simx_opmode_blocking);
disp('Atuador - Relativo')
disp(['Posicao: '    , num2str(j7_pos)])
disp(['Angulos Euler: ', num2str(j7_eul*180/pi)])
disp(' ')



[returnCode, actuator_pos]  =    sim.simxGetObjectPosition(clientID, atuador, ref, sim.simx_opmode_blocking);
[returnCode, actuator_eul]  = sim.simxGetObjectOrientation(clientID, atuador, ref, sim.simx_opmode_blocking);
disp('Atuador - Base')
disp(['Posicao: '    , num2str(actuator_pos)])
disp(['Angulos Euler: ', num2str(actuator_eul*180/pi)])
disp(' ')



disp('Posicao Relativa')
% Posições relativas
pos = [ j1_pos;
        j2_pos;
        j3_pos;
        j4_pos;
        j5_pos;
        j6_pos;
        j7_pos];

for i=1:7
    fprintf("\t%.10f, \t%.10f, \t%.10f\n", pos(i,1), pos(i,2), pos(i,3))
end
disp(' ')


disp('Angulacao relativa')
% Angulações relativas
ang = [ j1_eul;
        j2_eul;
        j3_eul;
        j4_eul;
        j5_eul;
        j6_eul;
        j7_eul]*180/pi;


pos = double(pos);
ang = double(ang);

for i=1:7
    fprintf("\t%.1f, \t%.1f, \t%.1f\n", ang(i,1), ang(i,2), ang(i,3))
end
disp(' ')


save("UR5_PosAng.mat", "pos","ang")
