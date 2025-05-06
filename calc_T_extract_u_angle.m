function [axis, angle, p] = calc_T_extract_u_angle(T)
    % CALC_T_EXTRACT_U_ANGLE extrai o eixo de rotação, o ângulo de rotação,
    % e o vetor de posição a partir de uma dada matriz de transformação homogênea T.
    %
    % Entradas:
    % - T: matriz de transformação homogênea 4x4
    %
    % Saídas:
    % - axis: vetor unitário 3x1 representando o eixo de rotação
    % - angle: Escalar representando o ângulo de rotação (em radianos)
    % - p: vetor de posição 3x1 [px, py, pz]

    % Valida as dimensões da matriz de entrada
    if size(T, 1) ~= 4 || size(T, 2) ~= 4
        error('Input matrix T must be a 4x4 homogeneous transformation matrix.');
    end

    % Extrai o vetor de posição da última coluna de T
    p = T(1:3, 4);

    % Extrai a matriz de rotação do bloco 3x3 no canto superior esquerdo de T
    R = T(1:3, 1:3);

    % Calcula o ângulo de rotação a partir do traço de R
    angle = acos((trace(R) - 1) / 2);

    % Trata casos especiais para o ângulo
    if abs(angle) < 1e-6
        % O ângulo é aproximadamente 0: sem rotação, eixo arbitrário
        axis = [1; 0; 0];
    elseif abs(angle - pi) < 1e-6
        % O ângulo é aproximadamente pi: caso especial para o eixo
        % Extrair os elementos diagonais

        diagR = diag(R);
        axis = sqrt((diagR + 1) / 2);

        % Ajustar os sinais com base nos elementos fora da diagonal
        if R(1, 2) < 0, axis(2) = -axis(2); end
        if R(1, 3) < 0, axis(3) = -axis(3); end
    else
        % Caso geral: calcular o eixo a partir dos elementos fora da diagonal
        axis = [R(3, 2) - R(2, 3);
                R(1, 3) - R(3, 1);
                R(2, 1) - R(1, 2)] / (2 * sin(angle));
    end

    % Garantir que o eixo seja um vetor unitário
    axis = axis / norm(axis);
end
