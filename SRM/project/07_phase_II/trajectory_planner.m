function [p_d, v_d] = trajectory_planner(t)
    %#codegen
    assert(isa(t, 'double'));
    assert(all(size(t) == [1, 1]));
    
    p_d = zeros(3, 1);
    v_d = zeros(3, 1);
    
    % Reachable Targets (Trocar at [-0.5; 0.0; 0.4], L = 0.15m)
    m0 = [-0.50;  0.00; 0.30]; % Starting position (10.0 cm from trocar)
    m1 = [-0.50;  0.08; 0.28]; % Target 1 (14.4 cm from trocar)
    m2 = [-0.50; -0.08; 0.28]; % Target 2 (14.4 cm from trocar)
    m3 = [-0.55;  0.00; 0.30]; % Target 3 (11.2 cm from trocar)
    
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
