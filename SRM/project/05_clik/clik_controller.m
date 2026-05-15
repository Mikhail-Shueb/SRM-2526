
% CLIK_CONTROLLER - Closed-loop inverse kinematics.
% We gonna use two methods:
% method = 1: damped pseudoinverse, with null-space projection
% method = 2: transposed Jacobian

function [q_dot, x_err, e_p, e_o] = clik_controller(q, p_d, R_d, p_dot_d, omega_d, Kp, Ko, q_rest, Kn, method, lambda)
    

if nargin < 10 || isempty(method)
    method = 1;
end
if nargin < 11 || isempty(lambda)
    lambda = 1e-3;
end

q = q(:);
p_d = p_d(:);
p_dot_d = p_dot_d(:);
omega_d = omega_d(:);
q_rest = q_rest(:);

if isscalar(Kp)
    Kp = Kp * eye(3);
end
if isscalar(Ko)
    Ko = Ko * eye(3);
end
if isscalar(Kn)
    Kn = Kn * eye(7);
end

T = kuka_direct_kinematics(q);
R = T(1:3, 1:3);
p = T(1:3, 4);
J = jacobian_kuka(q);

e_p = p_d - p;
e_o = orientation_error_axis_angle(R_d, R);

x_dot_ref = [p_dot_d + Kp * e_p;
             omega_d + Ko * e_o];

q_dot_0 = -Kn * (q - q_rest);

if method == 2
    q_dot = J' * x_dot_ref + q_dot_0;
else
    J_hash = damped_pseudoinverse(J, lambda);
    null_projector = eye(7) - J_hash * J;
    q_dot = J_hash * x_dot_ref + null_projector * q_dot_0;
end

x_err = [e_p; e_o];

end

function J_hash = damped_pseudoinverse(J, lambda)
JJt = J * J';
J_hash = J' / (JJt + (lambda^2) * eye(size(JJt)));
end

function e_o = orientation_error_axis_angle(R_d, R)
R_err = R_d * R';
cos_theta = (trace(R_err) - 1) / 2;
cos_theta = max(-1, min(1, cos_theta));
theta = acos(cos_theta);

if theta < 1e-8
    e_o = 0.5 * vee(R_err - R_err');
elseif abs(pi - theta) < 1e-5
    axis = sqrt(max(0, [R_err(1,1); R_err(2,2); R_err(3,3)] + 1) / 2);
    if R_err(3,2) - R_err(2,3) < 0, axis(1) = -axis(1); end
    if R_err(1,3) - R_err(3,1) < 0, axis(2) = -axis(2); end
    if R_err(2,1) - R_err(1,2) < 0, axis(3) = -axis(3); end
    if norm(axis) < 1e-8
        axis = [1; 0; 0];
    else
        axis = axis / norm(axis);
    end
    e_o = theta * axis;
else
    axis = vee(R_err - R_err') / (2 * sin(theta));
    e_o = theta * axis;
end
end

function v = vee(S)
v = [S(3,2); S(1,3); S(2,1)];
end
