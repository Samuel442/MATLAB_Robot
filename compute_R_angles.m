function R = compute_R_angles(angles)
    % COMP_R_ANGLES reconstructs a rotation matrix from Euler angles
    % using the ZYX convention.
    %
    % Input:
    % - angles: 3x1 vector [phi; theta; psi] (Euler angles)
    %
    % Output:
    % - R: 3x3 rotation matrix

    phi   = angles(1);   % Roll
    theta = angles(2); % Pitch
    psi   = angles(3);   % Yaw

    % Rotation matrix around x-axis (Roll)
    Rx = [1, 0, 0;
          0, cos(phi), -sin(phi);
          0, sin(phi), cos(phi)];

    % Rotation matrix around y-axis (Pitch)
    Ry = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];

    % Rotation matrix around z-axis (Yaw)
    Rz = [cos(psi), -sin(psi), 0;
          sin(psi), cos(psi), 0;
          0, 0, 1];

    % Combine rotations
    R = Rz * Ry * Rx;
end