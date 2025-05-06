function [p, alpha] = calc_T_extract_pos_ang(T)
    % EXTRACT_VECTORS_FROM_T extrai o vetor de posição e os ângulos de Euler
    % a partir de uma dada matriz de transformação homogênea T.
    %
    % Entradas:
    % - T: matriz de transformação homogênea 4x4
    %
    % Saidas:
    % - p: vetor de posicao 3x1 [tx, ty, tz]
    % - alpha: vetor de rotação 3x1 [phi, theta, psi] (angulos de Euler em radianos)

    % Validar as dimensoes da matriz de entrada
    if size(T,1) ~= 4 || size(T,2) ~= 4
        error('Input matrix T must be a 4x4 homogeneous transformation matrix.');
    end

    % Extrair o vetor de posição da última coluna de T
    p = T(1:3, 4);

    % Extrair a matriz de rotação do bloco 3x3 superior esquerdo de T
    R = T(1:3, 1:3);

    % alpha = compute_euler_angles_optimization(R);

    % Calcular os ângulos de Euler a partir da matriz de rotação
    % Usando a convenção ZYX: Rz * Ry * Rx

    theta = real(-asin(R(3,1)));

    % Verificar por bloqueio de giroscópio (singularidades)
    if abs(cos(theta)) > 1e-6
        % Caso geral: no gimbal lock
        phi = atan2(R(3,2), R(3,3)); % Roll
        psi = atan2(R(2,1), R(1,1)); % Yaw
    else
        % Caso Gimbal lock: theta = +-pi/2
        phi = atan2(-R(2,3), R(2,2)); % Roll
        psi = 0; % Yaw cannot be determined, set to 0
    end

    % % Combinar os ângulos de Euler no vetor de rotação
    alpha = [phi; theta; psi];
end





