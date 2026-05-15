% Inverse kinematics

function q = inverse_kinematics(R_d, p_d, psi)
% Inputs:
%   R_d : desired end-effector rotation
%   p_d : desired end-effector position in meters
%   psi : arm angle used to pick one redundant solution
%
% Output:
%   q   : joint vector [q1; ...; q7] rad

% Robot constants
d3 = 0.400;   % shoulder to elbow
d5 = 0.400;   % elbow to wrist centre
d7 = 0.126;   % wrist centre to end-effector

% Position part: first solve the arm up to the wrist centre.

% Wrist centre. The tool axis is the third column of R_d.
z_ee = R_d(:, 3);
p_w  = p_d - d7 * z_ee;

% Shoulder is at the base origin in this DH model.
p_s = [0; 0; 0];

v_sw = p_w - p_s;
r    = norm(v_sw);

% Keep acos arguments in range if the target is just outside the workspace.
if r > (d3 + d5)
    warning('IK: target is OUT OF REACH (r=%.4f > %.4f). Clamping.', r, d3+d5);
    r = d3 + d5;
end
if r < abs(d3 - d5)
    warning('IK: target is TOO CLOSE (r=%.4f < %.4f). Clamping.', r, abs(d3-d5));
    r = abs(d3 - d5);
end

% Elbow angle from the shoulder-elbow-wrist triangle.
cos_q4 = (r^2 - d3^2 - d5^2) / (2 * d3 * d5);
cos_q4 = max(-1, min(1, cos_q4));
q4     = acos(cos_q4);

n_sw = v_sw / norm(v_sw);

% Reference plane for the elbow position. psi rotates inside this plane.
e_z = [0; 0; 1];
if abs(dot(n_sw, e_z)) < 0.99
    t_ref = e_z - dot(e_z, n_sw) * n_sw;
else
    e_x   = [1; 0; 0];
    t_ref = e_x - dot(e_x, n_sw) * n_sw;
end
t_ref = t_ref / norm(t_ref);
b_ref = cross(n_sw, t_ref);

% Shoulder angle, same triangle.
cos_as = (d3^2 + r^2 - d5^2) / (2 * d3 * r);
cos_as = max(-1, min(1, cos_as));
angle_s = acos(cos_as);

n_se    = cos(angle_s) * n_sw + sin(angle_s) * (cos(psi)*t_ref + sin(psi)*b_ref);
p_elbow = p_s + d3 * n_se;

% From the DH table:
% z2 = [-cos(q1)sin(q2); -sin(q1)sin(q2); cos(q2)]
v_se_n = n_se;

q2 = acos(max(-1, min(1, v_se_n(3))));
sin_q2 = sin(q2);

if sin_q2 > 1e-6
    q1 = atan2(-v_se_n(2), -v_se_n(1));
else
    q1 = 0; % shoulder singularity
end

% q3 sets the elbow frame toward the wrist centre.
R01 = Rz(q1) * Rx(pi/2);
R12 = Rz(q2) * Rx(-pi/2);
R02 = R01 * R12;

v_ew_frame2 = R02' * (p_w - p_elbow);

% In frame 2, z4 has x/y terms with q3.
q3 = atan2(v_ew_frame2(2), v_ew_frame2(1));

% Orientation part: solve the spherical wrist.

R23 = Rz(q3) * Rx(-pi/2);
R34 = Rz(q4) * Rx(pi/2);
R04 = R02 * R23 * R34;

% Rotation still left for joints 5, 6 and 7.
R47 = R04' * R_d;

% Element comparison for the wrist:
% R47(3,3) = cos(q6)
% R47(1,3) = -cos(q5)*sin(q6)
% R47(2,3) = -sin(q5)*sin(q6)
% R47(3,1) = sin(q6)*cos(q7)
% R47(3,2) = -sin(q6)*sin(q7)

q6 = atan2(sqrt(R47(1,3)^2 + R47(2,3)^2), R47(3,3));

if abs(sin(q6)) > 1e-6   % normal case - no wrist singularity
    q5 = atan2(-R47(2,3), -R47(1,3));
    q7 = atan2(-R47(3,2),  R47(3,1));
else
    % Wrist singularity: choose q5 = 0 and put the rotation in q7.
    q5 = 0;
    q7 = atan2(-R47(2,1), R47(1,1));
end

q = [q1; q2; q3; q4; q5; q6; q7];

end  % inverse_kinematics


function R = Rz(angle)
    R = [cos(angle) -sin(angle) 0;
         sin(angle)  cos(angle) 0;
         0           0          1];
end

function R = Rx(angle)
    R = [1  0          0;
         0  cos(angle) -sin(angle);
         0  sin(angle)  cos(angle)];
end
