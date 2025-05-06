% Objetiva validar a determinação das matrizes de transformação T, usando
% as coordenadas homogêneas
clear, clc, close all


load('UR5_PosAng.mat')

pos_rel = round([pos, ang],6)

tam = size(pos_rel,1);


T = zeros(4,4,tam);

p0 = [0, 0, 0, 1]';
px = [1, 0, 0, 1]';
py = [0, 1, 0, 1]';
pz = [0, 0, 1, 1]';


for i = 1:tam
    pos_i = pos_rel(i,1:3);
    ang_i = pos_rel(i,4:6)*pi/180;

    T(:,:,i) = calc_T_pos_ang(pos_i, ang_i);

    Ti = eye(4);
    for k = 1:i
        Ti = Ti*T(:,:,k);
    end

    i 

    Ti

    % Conversão para o sistema global de posição
    P0 = (Ti*p0)' 
    Px = (Ti*px)'
    Py = (Ti*py)'
    Pz = (Ti*pz)'

    pause
end


