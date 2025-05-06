function T = calc_T_u_angle(angle, axis, p)
    % CALC_T_U_ANGLE calcula a matriz de transformação homogênea
    % a partir de um vetor de posição, eixo de rotação e ângulo de rotação.
    %
    % Entradas:
    % - p: vetor de posição 3x1 [px, py, pz]
    % - axis: vetor unitário 3x1 representando o eixo de rotação
    % - angle: escalar representando o ângulo de rotação (em radianos)
    %
    % Saída:
    % - T: matriz de transformação homogênea 4x4


    % Valide as dimensões da entrada
    if ~exist('p','var')
        p = zeros(3,1);
    elseif length(p) ~= 3 
        error('O vetor de posição p deve ser um vetor 3x1.');
    end

    if length(axis) ~= 3
        error('O eixo de rotação deve ser um vetor 3x1.');
    end

    % Garantir que o eixo seja um vetor unitário
    axis = axis / norm(axis);

    % Extrair os componentes do eixo
    ux = axis(1);
    uy = axis(2);
    uz = axis(3);

    % Define a matriz anti-simétrica de u
    U_cross = [   0, -uz,  uy;
                 uz,   0, -ux;
                -uy,  ux,   0  ];

    % Calcular a matriz de rotação utilizando a fórmula de rotação de Rodrigues
    R = eye(3) + sin(angle) * U_cross + (1 - cos(angle)) * (U_cross * U_cross);

    % Constroi a matriz de transformação homogênea
    T = [R, p(:);  % p(:) ensures p is a column vector
         0, 0, 0, 1];
end
