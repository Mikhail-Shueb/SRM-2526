function [p_d, v_d] = trajectory_planner(t)
    p_d = zeros(3, 1);
    v_d = zeros(3, 1);
    % Targets
    m0 = [-0.60;  0.00; 0.30]; % Starting position
    m1 = [-0.60;  0.10; 0.25];
    m2 = [-0.60; -0.10; 0.25];
    m3 = [-0.65;  0.00; 0.30];
    
    T_seg = 5.0; % 5 seconds per segment
    
    if t < T_seg
        p_start = m0; p_end = m1;
        tau = t;
    elseif t < 2*T_seg
        p_start = m1; p_end = m2;
        tau = t - T_seg;
    elseif t < 3*T_seg
        p_start = m2; p_end = m3;
        tau = t - 2*T_seg;
    else
        p_start = m3; p_end = m3;
        tau = T_seg;
    end
    
    % Cubic polynomial s(tau) from 0 to 1
    if tau <= 0
        s = 0; s_dot = 0;
    elseif tau >= T_seg
        s = 1; s_dot = 0;
    else
        s = 3*(tau/T_seg)^2 - 2*(tau/T_seg)^3;
        s_dot = (6*tau/(T_seg^2)) - (6*tau^2/(T_seg^3));
    end
    
    p_d = p_start + (p_end - p_start) * s;
    v_d = (p_end - p_start) * s_dot;
end
