% Obtenção da Matriz T a partir dos parâmetros de posição relativa
% obitdas com as medidas internas do próprio software
clc, clear, close all

load('UR5_PosAng.mat')

pos_rel = [pos, ang];
% pos_rel = round([pos, ang],6)

T_static = [];

for i = 1:size(pos_rel,1)
    i
    % Calcula a matriz homogênea usando um vetor p e ângulos phi, theta.
    % psi
    p = pos_rel(i,1:3);
    ang = pos_rel(i,4:6);
    T = calc_T_pos_ang(p, ang*pi/180)
    T_static(:,:,i) = T;

    [p, alpha] = calc_T_extract_pos_ang(T);

    disp(num2str(pos_rel(i,:),6))
    disp(num2str([p, alpha*180/pi],6))
    % pos_rel2 = [p, alpha*180/pi]
end

% save("UR5_T_static.mat", "T_static")

