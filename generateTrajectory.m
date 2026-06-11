function result = generateTrajectory(robot, target_xyz, toolSpeed, gainScale, curveType)
%% generateTrajectory  Build a joint-space trajectory to a single Cartesian target.
%%
%%   result = generateTrajectory(robot, target_xyz, toolSpeed, gainScale, curveType)
%%
%%   Inputs
%%     robot       — loadrobot('kinovaGen3','DataFormat','column') with Gravity set
%%     target_xyz  — [x y z] in metres, EE pointing DOWN
%%     toolSpeed   — scalar m/s  (e.g. 0.20)
%%     gainScale   — CTC gain multiplier (e.g. 3.0)
%%     curveType   — string: 'cubic'|'quintic'|'lspb'|'hermite'|'bangbang'
%%
%%   Output  result struct with fields:
%%     .q_ref_traj   (N x 8) [time, q1..q7]
%%     .qd_ref_traj  (N x 8)
%%     .tau_ref_traj (N x 8)  — accelerations (rad/s²) for CTC Add4
%%     .T_total      scalar
%%     .ik_err_mm    IK error in mm
%%     .Kp           (7x1)
%%     .Kd           (7x1)

if nargin < 5; curveType = 'cubic'; end

ee         = 'EndEffector_Link';
tauMax     = [187;187;187;52;52;52;52];
dt         = 0.001;
tx = target_xyz(1); ty = target_xyz(2); tz = target_xyz(3);

q0 = homeConfiguration(robot);
T0 = getTransform(robot, q0, ee);

%% IK
taskFinal = trvec2tform([tx ty tz]) * axang2tform([0 1 0 pi]);
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.MaxNumIteration   = 1500;
q_target = wrapToPi(ik(ee, taskFinal, [1 1 1 1 1 1], q0));

T1 = getTransform(robot, q_target, ee);
result.ik_err_mm = norm(T1(1:3,4) - [tx;ty;tz]) * 1000;
if result.ik_err_mm > 5
    error('generateTrajectory: IK error %.1f mm > 5 mm — target unreachable', result.ik_err_mm);
end

%% Timing
distance       = norm(tform2trvec(T0) - tform2trvec(taskFinal));
T_total        = max(round(distance/toolSpeed, 1), 2.0);
timestamp      = (0:dt:T_total)';
N              = length(timestamp);
result.T_total = T_total;

%% Trajectory profiles
pos = zeros(7,N); vel = zeros(7,N); acc = zeros(7,N);
for j = 1:7
    qs = q0(j); qe = q_target(j);
    t  = timestamp';
    switch lower(curveType)
        case 'quintic'
            T = T_total;
            a3_ =  10*(qe-qs)/T^3;
            a4_ = -15*(qe-qs)/T^4;
            a5_ =   6*(qe-qs)/T^5;
            pos(j,:) = qs + a3_.*t.^3 + a4_.*t.^4 + a5_.*t.^5;
            vel(j,:) = 3*a3_.*t.^2 + 4*a4_.*t.^3 + 5*a5_.*t.^4;
            acc(j,:) = 6*a3_.*t    +12*a4_.*t.^2 +20*a5_.*t.^3;
        case 'lspb'
            tb = T_total/4; T = T_total;
            av = (qe-qs)/(tb*(T-tb));
            for k=1:N
                tk=t(k);
                if tk<tb
                    pos(j,k)=qs+0.5*av*tk^2; vel(j,k)=av*tk; acc(j,k)=av;
                elseif tk<T-tb
                    pos(j,k)=qs+av*tb*(tk-tb/2); vel(j,k)=av*tb; acc(j,k)=0;
                else
                    pos(j,k)=qe-0.5*av*(T-tk)^2; vel(j,k)=av*(T-tk); acc(j,k)=-av;
                end
            end
        case 'bangbang'
            T=T_total; tm=T/2; av=4*(qe-qs)/T^2;
            for k=1:N
                tk=t(k);
                if tk<=tm; acc(j,k)=av; vel(j,k)=av*tk; pos(j,k)=qs+0.5*av*tk^2;
                else; acc(j,k)=-av; vel(j,k)=av*(T-tk); pos(j,k)=qe-0.5*av*(T-tk)^2; end
            end
        otherwise  %% cubic (default) and hermite
            a2_ =  3*(qe-qs)/T_total^2;
            a3_ = -2*(qe-qs)/T_total^3;
            pos(j,:) = qs + a2_.*t.^2 + a3_.*t.^3;
            vel(j,:) = 2*a2_.*t + 3*a3_.*t.^2;
            acc(j,:) = 2*a2_    + 6*a3_.*t;
    end
end

%% Gains
Kp_base = [100;100;80;60;40;40;20];
Kd_base = [20;20;16;12;8;8;4];
result.Kp = Kp_base * gainScale;
result.Kd = Kd_base * gainScale;

%% Package
result.q_ref_traj   = [timestamp, pos'];
result.qd_ref_traj  = [timestamp, vel'];
result.tau_ref_traj = [timestamp, acc'];
end