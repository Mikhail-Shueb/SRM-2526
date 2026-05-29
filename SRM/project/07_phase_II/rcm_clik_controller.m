function q_dot = rcm_clik_controller(q, p_d, v_d, c_r)
    %#codegen
    assert(isa(q, 'double'));
    assert(all(size(q) == [7, 1]));
    assert(isa(p_d, 'double'));
    assert(all(size(p_d) == [3, 1]));
    assert(isa(v_d, 'double'));
    assert(all(size(v_d) == [3, 1]));
    assert(isa(c_r, 'double'));
    assert(all(size(c_r) == [3, 1]));

    q_dot = zeros(7, 1);
    % Gains
    K_rcm = 10;
    K_tool = 5;
    
    % Forward Kinematics
    T_e = kuka_direct_kinematics(q);
    p_e = T_e(1:3, 4);
    R_e = T_e(1:3, 1:3);
    z_e = R_e(:, 3);
    
    J_e = jacobian_kuka(q);
    J_v = J_e(1:3, :);
    J_w = J_e(4:6, :);
    
    % Tool Kinematics
    L = 0.15;
    p_tool = p_e + L * z_e;
    skew_Lz = [0, -L*z_e(3), L*z_e(2);
               L*z_e(3), 0, -L*z_e(1);
               -L*z_e(2), L*z_e(1), 0];
    J_tool = J_v - skew_Lz * J_w;
    
    % RCM Kinematics
    lambda = (c_r - p_e)' * z_e;
    p_c = p_e + lambda * z_e;
    e_rcm = c_r - p_c;
    
    skew_lamz = [0, -lambda*z_e(3), lambda*z_e(2);
                 lambda*z_e(3), 0, -lambda*z_e(1);
                 -lambda*z_e(2), lambda*z_e(1), 0];
    J_pc = J_v - skew_lamz * J_w;
    
    Proj_ortho = eye(3) - z_e * z_e';
    J_rcm = Proj_ortho * J_pc;
    
    % Task-Priority CLIK
    % Task 1: RCM Constraint (highest priority)
    J_rcm_pinv = pinv(J_rcm);
    q_dot_1 = J_rcm_pinv * (K_rcm * e_rcm);
    
    % Task 2: Tool tracking (secondary priority)
    e_tool = p_d - p_tool;
    v_tool_1 = J_tool * q_dot_1;
    v_tool_req = v_d + K_tool * e_tool - v_tool_1;
    
    N_1 = eye(7) - J_rcm_pinv * J_rcm;
    J_tool_N = J_tool * N_1;
    
    % Damped Least Squares for Secondary Task to avoid singularities
    lambda_damp = 0.1;
    J_tool_N_pinv = J_tool_N' / (J_tool_N * J_tool_N' + lambda_damp^2 * eye(3));
    
    q_dot_2 = J_tool_N_pinv * v_tool_req;
    
    q_dot = q_dot_1 + q_dot_2;
end
