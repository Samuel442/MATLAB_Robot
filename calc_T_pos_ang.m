function [T] = calc_T_pos_ang(p, alpha)
    % CALC_T_POS_ANG calcula a matriz de transformação homogênea
    % dada um vetor de posição p e um vetor de rotação alpha (ângulos de Euler).
    %
    % Entradas:
    % - p: vetor de posição 3x1 [px, py, pz]
    % - alpha: vetor de rotação 3x1 [phi, theta, psi] (ângulos de Euler em radianos)
    %
    % Saída:
    % - T: matriz de transformação homogênea 4x4

    % Extrair os componentes da posição
    px = p(1);
    py = p(2);
    pz = p(3);

    % Extrair os ângulos de Euler (roll, pitch, yaw)
    phi = alpha(1);    % Roll (rotação ao redor do eixo x)
    theta = alpha(2);  % Pitch (rotação ao redor do eixo y)
    psi = alpha(3);    % Yaw (rotação ao redor do eixo z)

    % Matriz de rotação ao redor do eixo x (rolamento)
    Rx = [1, 0, 0;
          0, cos(phi), -sin(phi);
          0, sin(phi), cos(phi)];
    
    % Matriz de rotação ao redor do eixo y (inclinação)
    Ry = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];
    
    % Matriz de rotação ao redor do eixo z (guinada)
    Rz = [cos(psi), -sin(psi), 0;
          sin(psi), cos(psi), 0;
          0, 0, 1];

    % Matriz de rotação combinada R = Rz * Ry * Rx
    R = Rz * Ry * Rx;

    % Constroi a matriz de transformação homogênea
    T = [R, [px; py; pz];
         0, 0, 0, 1];
end


