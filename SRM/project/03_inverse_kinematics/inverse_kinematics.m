function q = inverse_kinematics(R_d, p_d, psi)
%INVERSE_KINEMATICS  Closed-form IK for KUKA LBR MED using kinematic decoupling.
%
%  INPUTS:
%    R_d  : 3x3 desired rotation matrix of the end-effector (orientation goal)
%    p_d  : 3x1 desired position of the end-effector [metres]
%    psi  : arm angle in radians (resolves the 7-DOF redundancy).
%           psi=0 means the elbow stays in the default reference plane.
%
%  OUTPUT:
%    q    : 7x1 vector of joint angles [q1;q2;q3;q4;q5;q6;q7] in radians
%
%  METHOD: Kinematic Decoupling
%    Step A - POSITION problem  -> finds q1, q2, q3, q4
%    Step B - ORIENTATION problem -> finds q5, q6, q7

% ---- Robot constants (metres) ----------------------------------------
% d1 is 0 because base frame and frame 1 share the same origin.
d3 = 0.400;   % shoulder to elbow
d5 = 0.400;   % elbow to wrist centre
d7 = 0.126;   % wrist centre to end-effector

% ======================================================================
% STEP A: POSITION - find where the wrist centre must be,
%         then solve for q1, q2, q3 (shoulder) and q4 (elbow).
% ======================================================================

% --- A1. Compute the wrist centre position ----------------------------
% The last z-axis of the end-effector (R_d's third column) points along
% the tool. Walking back d7 along it gives the wrist centre p_w.
z_ee = R_d(:, 3);
p_w  = p_d - d7 * z_ee;

% --- A2. Shoulder position (fixed point in space) ---------------------
% Since d1 = 0 in the DH table, the shoulder is at the base origin.
p_s = [0; 0; 0];

% --- A3. Vector and distance from shoulder to wrist -------------------
v_sw = p_w - p_s;
r    = norm(v_sw);

% --- A4. Reachability check -------------------------------------------
if r > (d3 + d5)
    warning('IK: target is OUT OF REACH (r=%.4f > %.4f). Clamping.', r, d3+d5);
    r = d3 + d5;
end
if r < abs(d3 - d5)
    warning('IK: target is TOO CLOSE (r=%.4f < %.4f). Clamping.', r, abs(d3-d5));
    r = abs(d3 - d5);
end

% --- A5. Solve q4 - the elbow angle (law of cosines) -----------------
% Triangle: shoulder - elbow - wrist, sides d3, d5, r.
cos_q4 = (r^2 - d3^2 - d5^2) / (2 * d3 * d5);
cos_q4 = max(-1, min(1, cos_q4));   % clamp for numerical safety
q4     = acos(cos_q4);              % elbow-up solution (positive)

% --- A6. Find the elbow position using the arm angle psi --------------
n_sw = v_sw / norm(v_sw); % strictly unit vector

% Build a reference vector t_ref perpendicular to n_sw
e_z = [0; 0; 1];
if abs(dot(n_sw, e_z)) < 0.99          % n_sw is not parallel to Z
    t_ref = e_z - dot(e_z, n_sw) * n_sw;
else                                   % n_sw nearly parallel to Z - use X
    e_x   = [1; 0; 0];
    t_ref = e_x - dot(e_x, n_sw) * n_sw;
end
t_ref = t_ref / norm(t_ref);
b_ref = cross(n_sw, t_ref);            % completes the right-hand frame

% Angle at shoulder (law of cosines in the same triangle)
cos_as = (d3^2 + r^2 - d5^2) / (2 * d3 * r);
cos_as = max(-1, min(1, cos_as));
angle_s = acos(cos_as);

% Direction from shoulder to elbow (depends on psi)
n_se    = cos(angle_s) * n_sw + sin(angle_s) * (cos(psi)*t_ref + sin(psi)*b_ref);
p_elbow = p_s + d3 * n_se;            % actual elbow position in world

% --- A7. Solve q1 and q2 from the shoulder-to-elbow direction ---------
% Based on the new DH parameters (alpha1 = pi/2, alpha2 = -pi/2), the Z2 axis in 
% frame 0 is: Z2 = [-cos(q1)*sin(q2); -sin(q1)*sin(q2); cos(q2)]
% Since the elbow position is p_elbow = d3 * Z2, we can set Z2 = n_se.
v_se_n = n_se;   % already unit length

% We assume q2 is between 0 and pi (so sin(q2) >= 0)
q2 = acos(max(-1, min(1, v_se_n(3))));
sin_q2 = sin(q2);

if sin_q2 > 1e-6
    q1 = atan2(-v_se_n(2), -v_se_n(1));
else
    q1 = 0; % Singularity: shoulder straight up/down
end

% --- A8. Solve q3 - arm roll (maps psi to the DH joint angle) ---------
% We compute R_01*R_12 (=R02) then find q3 so that the elbow frame 
% aligns with the wrist centre direction.
R01 = Rz(q1) * Rx(pi/2);
R12 = Rz(q2) * Rx(-pi/2);
R02 = R01 * R12;

% The wrist centre seen from the elbow frame (frame 2)
v_ew_frame2 = R02' * (p_w - p_elbow);

% From DH math, v_ew_frame2 = d5 * Z4_in_frame2
% Where Z4_in_frame2 = [cos(q3)*sin(q4); sin(q3)*sin(q4); cos(q4)]
% So we can find q3 from the x and y components.
q3 = atan2(v_ew_frame2(2), v_ew_frame2(1));

% ======================================================================
% STEP B: ORIENTATION - given q1..q4, find q5, q6, q7
% ======================================================================

% --- B1. Compute the rotation matrix for frames 0 to 4 ---------------
R23 = Rz(q3) * Rx(-pi/2);
R34 = Rz(q4) * Rx(pi/2);
R04 = R02 * R23 * R34;

% --- B2. Remaining rotation to be achieved by the wrist (joints 5,6,7)
R47 = R04' * R_d;

% --- B3. Extract q5, q6, q7 using Euler angle decomposition -------
% Based on the new DH (alpha5=pi/2, alpha6=-pi/2, alpha7=0):
% R45 * R56 * R67 produces a specific ZYZ-like rotation matrix.
% Comparing the elements analytically:
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
    % Wrist singularity: q6 = 0 or pi, q5 and q7 are co-linear.
    % Set q5 = 0 and absorb everything into q7.
    q5 = 0;
    q7 = atan2(-R47(2,1), R47(1,1));
end

% ======================================================================
% Collect output
% ======================================================================
q = [q1; q2; q3; q4; q5; q6; q7];

end  % inverse_kinematics


% ======================================================================
% Helper: elementary rotation matrices
% ======================================================================
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
