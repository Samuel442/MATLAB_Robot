function euler_angles = compute_euler_angles_optimization(R)
    % COMPUTE_EULER_ANGLES_OPTIMIZATION finds the Euler angles that best match
    % a given rotation matrix using optimization.
    %
    % Input:
    % - R: 3x3 rotation matrix
    %
    % Output:
    % - euler_angles: 3x1 vector [phi; theta; psi] (Euler angles in radians)

    % Validate the input matrix dimensions
    if size(R, 1) ~= 3 || size(R, 2) ~= 3
        error('Input matrix R must be a 3x3 rotation matrix.');
    end

    % Define the objective function to minimize
    objective = @(angles) 1000*norm(R - compute_R_angles(angles), 'fro')^2;

    % Initial guess for the Euler angles
    initial_guess = [0; 0; 0]; % [phi; theta; psi]

    % Set optimization options
    options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'off');

    % Solve the optimization problem
    euler_angles = fminunc(objective, initial_guess, options);
end

