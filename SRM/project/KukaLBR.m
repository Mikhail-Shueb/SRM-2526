function Robot = KukaLBR()
%KUKALBR Returns the D-H table of parameters for the KUKA LBR MED 7 R800
%   Robot = [d v a alpha offset];
%
%   d: link offset along previous z to the common normal
%   v: joint angle around previous z (symbolic variables q1 to q7)
%   a: link length along common normal
%   alpha: link twist angle around common normal
%   offset: coordinate offset for home position
%
%   Note: The parameters represent the Standard Denavit-Hartenberg (DH) 
%   convention as expected by the Robotics Symbolic Matlab Toolbox.

% Define symbolic variables for the 7 rotational joints
syms q1 q2 q3 q4 q5 q6 q7 real

% KUKA LBR MED kinematics dimensions (in meters)
% These measurements correspond to the lengths of the robot links.
d1 = 0.340; % Base to Shoulder
d3 = 0.400; % Shoulder to Elbow
d5 = 0.400; % Elbow to Wrist
d7 = 0.126; % Wrist to Flange

% Define the Standard DH Table
% Each row corresponds to a joint from 1 to 7.
% Columns: [d      v       a       alpha       offset]
Robot = [  d1     q1      0       -pi/2         0;
           0      q2      0        pi/2         0;
           d3     q3      0       -pi/2         0;
           0      q4      0        pi/2         0;
           d5     q5      0       -pi/2         0;
           0      q6      0        pi/2         0;
           d7     q7      0        0            0 ];

end
