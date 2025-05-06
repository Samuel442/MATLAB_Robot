clear, clc, close all

load("UR5_T_static.mat");
% load("UR5_WayPoints.mat");

% % % WayPoints for the robot arm in configuration space

% wp_q_ref      =    [  0,   0,   0,   0,   0,   0;
%                     -90,   0,   0,   0,   0,   0;
%                       0, -90,   0,   0,   0,   0;
%                       0,   0, -90,   0,   0,   0;
%                       0,   0,   0, -90,   0,   0;
%                       0,   0,   0,   0, -90,   0;
%                       0,   0,   0,   0,   0, -90;
%                       0,   0,   0,   0,   0,   0;];


wp_q_ref      =    [    0,   0,   0,   0,    0,   0;
                       10,  15, -10, -15,  -20,  10;
                       80,  30,  45,  50,   20, 100;
                     -100, -50, -30,   0,  100,  -5;
                      -10,  30,  35,  25,   15,  50;
                      15,    0,  35, -50, -150, -90;
                      150,  75,  20,  10,   -5,  15;
                        0,   0,   0,   0,    0,   0];

%% 
tam = size(wp_q_ref,1);

% O eixo ao redor de cada ângulo que q gira
u_static = [ 0, 0, 1; 
             0, 0, 1; 
             0, 0, 1; 
             0, 0, 1; 
             0, 0, 1; 
             0, 0, 1]; 


q0 = zeros(1,6);

wp_q_opt   = [];
wp_3d_calc = [];

tic
for i = 1:tam
    % Obter a referência 
    q_ref = wp_q_ref(i,:);
    T_ref = calc_T_dynamic(T_static,q_ref*pi/180,u_static);    
    [u_ref, ang_ref, pos_ref] = calc_T_extract_u_angle(T_ref); 
    
    % Armazenar os valores de waypoint 3D calculados
    wp_3d_calc = [wp_3d_calc;
                  pos_ref', rad2deg(ang_ref'), u_ref'];
    
    q_star = ur5_IK_a_nonlinear(T_ref,T_static,q0);
    % q_star = ur5_IK_b_fmincon(T_ref,T_static,q0);
    % q_star = ur5_IK_c_partswarm(T_ref,T_static,q0);
    % q0 = q_star;
    
    % Waypoints otimizados para o espaço de configurações 
    wp_q_opt = [wp_q_opt; q_star];
    
    disp(' ')
    disp(['q_ref : ',num2str(q_ref)])
    disp(['q_star: ',num2str(q_star)])
end
toc

%% 

wp_q_ref
wp_q_opt

[wp_3d_opt, wp_3d_ref] = follow_waypoints_two(wp_q_opt,wp_q_ref, false);


wp_3d_opt
wp_3d_ref


% Erro de posição 
erro_pos = vecnorm(wp_3d_ref(:,1:3) - wp_3d_opt(:,1:3), 2, 2);

% Erro angular (em graus, considerando diferença entre ângulos de rotação)
erro_ang = vecnorm(wp_3d_ref(:,4:6) - wp_3d_opt(:,4:6), 2, 2);

% Total médio dos erros
media_erro_pos = mean(erro_pos);
media_erro_ang = mean(erro_ang);

fprintf('Erro médio de posição: %.4f\n', media_erro_pos);
fprintf('Erro médio de orientação (graus): %.4f\n', media_erro_ang);
