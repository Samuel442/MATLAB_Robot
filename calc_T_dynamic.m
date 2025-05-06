function T_global = calc_T_dynamic(T_static, angles_of_rotation, rotation_axis)

    T_global = eye(4);

    for i = 1:length(angles_of_rotation)
        T_dynamic = calc_T_u_angle(angles_of_rotation(i), rotation_axis(i,:));
        T_global = T_global * T_static(:,:,i) * T_dynamic;
    end

end



