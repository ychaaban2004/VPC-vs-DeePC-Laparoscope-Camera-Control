%% params.m — shared parameter file (plant + controller)
% FILE ALREADY REFERENCED IN MODEL PROPERTIES
% References paper for values, and equations and lists sources/justification of values

%% Timing
dt = 1/30;            % [s] sampling period δt (Eq. 6). Justification: see shared doc

%% Initial condition
q0 = [0; -pi/2; pi/2; -pi/2; -pi/2; 0];   % [rad] "tool down" pose. Justification: see shared doc

%% Robot Arm
ur5 = loadrobot("universalUR5", "DataFormat", "column");

% Laparoscope/camera attachment
L_scope = 0.30;     % [m] flange -> camera. Source: within typical laproscope length range
%example reference of typical equipment: https://www.medwrench.com/equipment/10830/stryker-ideal-eyes

% Camera body at the laproscope tip, along the flange z-axis (NOT in paper)
cam = rigidBody("camera");
setFixedTransform(cam.Joint, trvec2tform([0 0 L_scope]));
addBody(ur5, cam, "tool0");

%% Camera intrinsics (Eqs. 3-4)
img_w = [];         % [px] image width.  Source:
img_h = [];         % [px] image height. Source:
fx = [];            % [px] focal length x. Source:
fy = [];            % [px] focal length y. Source:
u0 = [];            % [px] principal point u. Source:
v0 = [];            % [px] principal point v. Source:

%% Depth (Eq. 3)
depth = [];         % [m] target depth d in camera frame. Source:

%% Disturbance / noise (Eq. 6)
Bd      = [];       % 2x2 disturbance distribution matrix. NOT in paper. Source:
sigma_w = [];       % std of w(k), Sec. IV-A. Source:

%% Joint limits (Eq. 7 constraints — enforced by CONTROLLER, not plant)
q_min    = [];      % [rad]   6x1. Source:
q_max    = [];      % [rad]   6x1. Source:
qdot_min = [];      % [rad/s] 6x1. Source:
qdot_max = [];      % [rad/s] 6x1. Source:

%% FoV safe set (Eq. 8)
s_c = [];           % [px] 2x1 FoV centre. Source:
r   = [];           % [px] FoV radius. Source:

