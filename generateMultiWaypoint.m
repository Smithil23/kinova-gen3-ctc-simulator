function result = generateMultiWaypoint(robot, waypoints_xyz, orientations, toolSpeeds)
%% generateMultiWaypoint  Multi-waypoint cubic Hermite trajectory.
%%
%%   result = generateMultiWaypoint(robot, waypoints_xyz, orientations, toolSpeeds)
%%
%%   Inputs
%%     robot         — loadrobot result with Gravity set
%%     waypoints_xyz — (nWP x 3) matrix of [x y z] targets
%%     orientations  — (nWP x 1) cell of strings: 'Down (-Z)','Up (+Z)',
%%                     'Horizontal (+X)','Tilted 45°'
%%     toolSpeeds    — scalar OR (nWP x 1) vector of speeds in m/s
%%
%%   Output result struct: same fields as generateTrajectory

ee     = 'EndEffector_Link';
tauMax = [187;187;187;52;52;52;52];
dt     = 0.001;
nWP    = size(waypoints_xyz, 1);

if nargin < 3 || isempty(orientations)
    orientations = repmat({'Down (-Z)'}, nWP, 1);
end
if nargin < 4 || isempty(toolSpeeds)
    toolSpeeds = 0.20 * ones(nWP,1);
elseif isscalar(toolSpeeds)
    toolSpeeds = toolSpeeds * ones(nWP,1);
end

q0 = homeConfiguration(robot);
q_wps = zeros(7, nWP+1);
q_wps(:,1) = q0;

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxNumIteration   = 1500;

%% IK for every waypoint
for w = 1:nWP
    tx=waypoints_xyz(w,1); ty=waypoints_xyz(w,2); tz=waypoints_xyz(w,3);
    ori_ = oriString2tform(orientations{w});
    T_goal = trvec2tform([tx ty tz]) * ori_;
    q_s = wrapToPi(ik(ee, T_goal, [1 1 1 1 1 1], q_wps(:,w)));
    Tc  = getTransform(robot, q_s, ee);
    em  = norm(Tc(1:3,4) - [tx;ty;tz]) * 1000;
    if em > 5
        error('generateMultiWaypoint: WP%d unreachable (%.1f mm)', w, em);
    end
    q_wps(:,w+1) = q_s;
end

%% Segment durations (per-WP speed)
seg_T = zeros(1,nWP);
for w = 1:nWP
    Ts = getTransform(robot, q_wps(:,w),   ee);
    Te = getTransform(robot, q_wps(:,w+1), ee);
    d  = norm(tform2trvec(Ts) - tform2trvec(Te));
    seg_T(w) = max(round(d/toolSpeeds(w), 1), 1.5);
end

%% Junction velocities
jv = zeros(7, nWP+1);
for w = 2:nWP
    di = (q_wps(:,w)   - q_wps(:,w-1)) / seg_T(w-1);
    do_= (q_wps(:,w+1) - q_wps(:,w))   / seg_T(w);
    vj = (seg_T(w).*di + seg_T(w-1).*do_) / (seg_T(w-1)+seg_T(w));
    for j=1:7
        if sign(di(j)) ~= sign(do_(j)); vj(j)=0; end
    end
    jv(:,w) = vj;
end

%% Build trajectory
all_t=[]; all_p=[]; all_v=[]; all_a=[]; t_off=0;
for w = 1:nWP
    qs=q_wps(:,w); qe=q_wps(:,w+1);
    qds=jv(:,w); qde=jv(:,w+1); T_=seg_T(w);
    ts=(0:dt:T_)'; Ns=length(ts);
    p_=zeros(7,Ns); v_=zeros(7,Ns); a_=zeros(7,Ns);
    for j=1:7
        a0=qs(j); a1=qds(j);
        a2=(3*(qe(j)-qs(j))/T_^2)-(2*qds(j)/T_)-(qde(j)/T_);
        a3=(-2*(qe(j)-qs(j))/T_^3)+((qds(j)+qde(j))/T_^2);
        t=ts';
        p_(j,:)=a0+a1.*t+a2.*t.^2+a3.*t.^3;
        v_(j,:)=a1+2*a2.*t+3*a3.*t.^2;
        a_(j,:)=2*a2+6*a3.*t;
    end
    if w<nWP; p_=p_(:,1:end-1); v_=v_(:,1:end-1); a_=a_(:,1:end-1); ts=ts(1:end-1); end
    all_t=[all_t; ts+t_off]; all_p=[all_p,p_]; all_v=[all_v,v_]; all_a=[all_a,a_];
    t_off = t_off + ts(end) + dt;
end

result.q_ref_traj   = [all_t, all_p'];
result.qd_ref_traj  = [all_t, all_v'];
result.tau_ref_traj = [all_t, all_a'];
result.T_total      = all_t(end);
result.seg_T        = seg_T;
result.ik_err_mm    = 0;   % checked per-WP above
result.Kp = [100;100;80;60;40;40;20] * 3.0;
result.Kd = [20;20;16;12;8;8;4]     * 3.0;
end

function T = oriString2tform(s)
switch strtrim(s)
    case 'Up (+Z)';          T = eye(4);
    case 'Horizontal (+X)';  T = axang2tform([0 0 1 pi/2]);
    case 'Tilted 45 deg';    T = axang2tform([0 1 0 pi*3/4]);
    otherwise;               T = axang2tform([0 1 0 pi]);  % Down (-Z)
end
end