function result = generateTrajectoryTaskSpace(robot, start_xyz, target_xyz, ...
                                          oriStart, oriEnd, toolSpeed)
%% generateTrajectoryTaskSpace  Cartesian straight-line path with SLERP orientation.
%%
%%   EE moves on a straight line in 3D space.
%%   Orientation is SLERP-interpolated from oriStart to oriEnd.
%%   Jacobian pseudo-inverse gives joint velocities/accelerations.
%%
%%   Inputs
%%     robot       — loadrobot result with Gravity set
%%     start_xyz   — [x y z] start position (use [] to use current home EE pos)
%%     target_xyz  — [x y z] end position
%%     oriStart    — string: 'Up (+Z)' | 'Down (-Z)' | etc.
%%     oriEnd      — string (same options)
%%     toolSpeed   — m/s along the path
%%
%%   Output  result struct: same fields as generateTrajectory

if nargin < 4; oriStart  = 'Up (+Z)';   end
if nargin < 5; oriEnd    = 'Down (-Z)'; end
if nargin < 6; toolSpeed = 0.10;        end

ee     = 'EndEffector_Link';
tauMax = [187;187;187;52;52;52;52];
dt     = 0.001;

q0 = homeConfiguration(robot);
T0 = getTransform(robot, q0, ee);

if isempty(start_xyz)
    p0 = tform2trvec(T0);
else
    p0 = start_xyz(:)';
end
p1 = target_xyz(:)';

%% Build orientation quaternions from strings
qrot0 = rotm2quat(oriStr2rotm(oriStart));
qrot1 = rotm2quat(oriStr2rotm(oriEnd));

%% Timing
distance  = norm(p1 - p0);
T_total   = max(round(distance/toolSpeed, 1), 2.0);
timestamp = (0:dt:T_total)';
N         = length(timestamp);

%% Scalar cubic profile  s: 0→1
a2_s =  3/T_total^2;
a3_s = -2/T_total^3;
t_vec = timestamp';
s_vec = a2_s.*t_vec.^2 + a3_s.*t_vec.^3;
ee_path = p0 + s_vec' .* (p1 - p0);   % (N x 3)

%% Verify target reachable
taskFinal = trvec2tform(p1) * oriStr2tform(oriEnd);
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxNumIteration   = 1500;
q_chk = wrapToPi(ik(ee, taskFinal, [1 1 1 1 1 1], q0));
Tc    = getTransform(robot, q_chk, ee);
ik_err = norm(Tc(1:3,4) - p1') * 1000;
if ik_err > 5
    error('generateTrajectoryTaskSpace: target unreachable (IK err %.1f mm)', ik_err);
end

%% IK at every timestep with SLERP orientation
positions = zeros(7, N);
q_prev    = q0;
for i = 1:N
    t_n   = s_vec(i);
    p_i   = ee_path(i,:);
    q_sl  = quatinterp(qrot0, qrot1, t_n, 'slerp');
    R_sl  = quat2rotm(q_sl);
    T_goal = eye(4);
    T_goal(1:3,1:3) = R_sl;
    T_goal(1:3,4)   = p_i';
    q_i   = wrapToPi(ik(ee, T_goal, [0 0 0 1 1 1], q_prev));
    Tc2   = getTransform(robot, q_i, ee);
    if norm(Tc2(1:3,4) - p_i') * 1000 < 10
        q_prev = q_i;
    end
    positions(:,i) = q_i;
end

%% Jacobian pseudo-inverse for velocities and accelerations
sd_vec  = (2*a2_s.*t_vec + 3*a3_s.*t_vec.^2);
sdd_vec = (2*a2_s         + 6*a3_s.*t_vec);
ee_vel = sd_vec'  .* (p1 - p0);
ee_acc = sdd_vec' .* (p1 - p0);

velocities    = zeros(7, N);
accelerations = zeros(7, N);
J_prev = zeros(6,7);
for i = 1:N
    J   = geometricJacobian(robot, positions(:,i), ee);
    Jv  = J(4:6,:);
    velocities(:,i) = pinv(Jv) * ee_vel(i,:)';
    if i > 1
        dJv_qd = (Jv - J_prev(4:6,:))/dt * velocities(:,i-1);
    else
        dJv_qd = zeros(3,1);
    end
    accelerations(:,i) = pinv(Jv) * (ee_acc(i,:)' - dJv_qd);
    J_prev = J;
end

result.q_ref_traj   = [timestamp, positions'];
result.qd_ref_traj  = [timestamp, velocities'];
result.tau_ref_traj = [timestamp, accelerations'];
result.T_total      = T_total;
result.ik_err_mm    = ik_err;
result.Kp = [100;100;80;60;40;40;20] * 3.0;
result.Kd = [20;20;16;12;8;8;4]     * 3.0;
end

function R = oriStr2rotm(s)
switch strtrim(s)
    case 'Up (+Z)';          R = eye(3);
    case 'Horizontal (+X)';  R = axang2rotm([0 0 1 pi/2]);
    case 'Tilted 45 deg';    R = axang2rotm([0 1 0 pi*3/4]);
    otherwise;               R = axang2rotm([0 1 0 pi]);
end
end

function T = oriStr2tform(s)
T = [oriStr2rotm(s), zeros(3,1); 0 0 0 1];
end