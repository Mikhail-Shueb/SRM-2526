function joint_velocities = rcm_clik_controller(joint_angles, desired_tooltip_pos, desired_tooltip_vel, trocar_pos)
    %#codegen
    assert(isa(joint_angles, 'double'));
    assert(all(size(joint_angles) == [7, 1]));
    assert(isa(desired_tooltip_pos, 'double'));
    assert(all(size(desired_tooltip_pos) == [3, 1]));
    assert(isa(desired_tooltip_vel, 'double'));
    assert(all(size(desired_tooltip_vel) == [3, 1]));
    assert(isa(trocar_pos, 'double'));
    assert(all(size(trocar_pos) == [3, 1]));

    joint_velocities = zeros(7, 1);
    
    % Controller Gains
    K_rcm = 10;
    K_tool = 20;
    
    % 1. Forward Kinematics (Flange Pose and Needle Orientation)
    flange_pose = kuka_direct_kinematics(joint_angles);
    flange_pos = flange_pose(1:3, 4);
    flange_rot = flange_pose(1:3, 1:3);
    needle_direction = flange_rot(:, 3); % Z-axis of the flange points along the needle
    
    % Flange Jacobians
    jacobian_flange = jacobian_kuka(joint_angles);
    flange_jac_linear = jacobian_flange(1:3, :);
    flange_jac_angular = jacobian_flange(4:6, :);
    
    % 2. Tool Kinematics (Tooltip Position and Jacobian)
    needle_length = 0.15; % 15 cm tool
    tooltip_pos = flange_pos + needle_length * needle_direction;
    
    skew_Lz = [0, -needle_length*needle_direction(3), needle_length*needle_direction(2);
               needle_length*needle_direction(3), 0, -needle_length*needle_direction(1);
               -needle_length*needle_direction(2), needle_length*needle_direction(1), 0];
    tooltip_jacobian = flange_jac_linear - skew_Lz * flange_jac_angular;
    
    % 3. RCM Kinematics (Trocar Constraint Error and Jacobian)
    insertion_depth = (trocar_pos - flange_pos)' * needle_direction;
    closest_point_on_shaft = flange_pos + insertion_depth * needle_direction;
    rcm_error = trocar_pos - closest_point_on_shaft;
    
    skew_lamz = [0, -insertion_depth*needle_direction(3), insertion_depth*needle_direction(2);
                 insertion_depth*needle_direction(3), 0, -insertion_depth*needle_direction(1);
                 -insertion_depth*needle_direction(2), insertion_depth*needle_direction(1), 0];
    closest_point_jacobian = flange_jac_linear - skew_lamz * flange_jac_angular;
    
    % Project Jacobian onto plane orthogonal to needle direction
    Proj_ortho = eye(3) - needle_direction * needle_direction';
    rcm_jacobian = Proj_ortho * closest_point_jacobian;
    
    % 4. Task-Priority Closed-Loop Inverse Kinematics (CLIK)
    
    % Task 1: RCM Trocar Constraint (Highest Priority)
    rcm_jacobian_pinv = pinv(rcm_jacobian);
    joint_vel_rcm = rcm_jacobian_pinv * (K_rcm * rcm_error);
    
    % Task 2: Tooltip Trajectory Tracking (Secondary Priority)
    tooltip_error = desired_tooltip_pos - tooltip_pos;
    tooltip_vel_from_rcm = tooltip_jacobian * joint_vel_rcm;
    tooltip_vel_required = desired_tooltip_vel + K_tool * tooltip_error - tooltip_vel_from_rcm;
    
    % Project tooltip tracking into RCM null-space to guarantee no trocar motion
    nullspace_projection = eye(7) - rcm_jacobian_pinv * rcm_jacobian;
    tooltip_jacobian_projected = tooltip_jacobian * nullspace_projection;
    
    % Damped Least Squares for Secondary Task to avoid singularities
    lambda_damp = 0.01;
    tooltip_jacobian_projected_pinv = tooltip_jacobian_projected' / (tooltip_jacobian_projected * tooltip_jacobian_projected' + lambda_damp^2 * eye(3));
    
    joint_vel_tooltip = tooltip_jacobian_projected_pinv * tooltip_vel_required;
    
    % Combine velocities
    joint_velocities = joint_vel_rcm + joint_vel_tooltip;
end
