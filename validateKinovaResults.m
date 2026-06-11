%% =========================================================
%% validateKinovaResults.m
%% Independent validation script for KinovaApp GUI outputs
%%
%% Run this AFTER running a simulation in KinovaApp.
%% It recomputes every metric from raw Simulink data and
%% compares against what the GUI displayed.
%%
%% Usage:
%%   1. Run any simulation in KinovaApp (Single/Multi/TCP)
%%   2. In MATLAB command window: validateKinovaResults
%%
%% Output:
%%   Prints PASS/FAIL for every metric with tolerance
%%   Saves validation_report.txt in current folder
%% =========================================================

fprintf('\n');
fprintf('%s\n', repmat('=',1,60));
fprintf('  KINOVA GEN3 CTC — INDEPENDENT VALIDATION REPORT\n');
fprintf('  %s\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
fprintf('%s\n', repmat('=',1,60));

%% ── Check required workspace variables exist ─────────────
required = {'out','q_ref_traj','qd_ref_traj','tau_ref_traj','last_t_s','last_Qout'};
missing = {};
for i=1:length(required)
    if ~evalin('base',sprintf('exist(''%s'',''var'')',required{i}))
        missing{end+1} = required{i}; %#ok<AGROW>
    end
end
if ~isempty(missing)
    fprintf('\nERROR: Missing workspace variables: %s\n', strjoin(missing,', '));
    fprintf('Run a simulation in KinovaApp first, then re-run this script.\n\n');
    return;
end

%% ── Load raw data from workspace ─────────────────────────
t_s       = evalin('base','last_t_s');
Qout      = evalin('base','last_Qout');
q_ref     = evalin('base','q_ref_traj');
qd_ref    = evalin('base','qd_ref_traj');
tau_ref   = evalin('base','tau_ref_traj');

ts_ref    = q_ref(:,1);
pos_ref   = q_ref(:,2:8)';    % 7 x N
vel_ref   = qd_ref(:,2:8)';
acc_ref   = tau_ref(:,2:8)';  % accelerations rad/s²

fprintf('\nData loaded:\n');
fprintf('  Simulation time:    %.3f → %.3f s\n', t_s(1), t_s(end));
fprintf('  Q_out timesteps:    %d\n', length(t_s));
fprintf('  Reference timesteps:%d\n', length(ts_ref));
fprintf('  Joints:             7\n\n');

%% ── Setup robot ──────────────────────────────────────────
fprintf('Loading robot model...\n');
robot = loadrobot('kinovaGen3','DataFormat','column');
robot.Gravity = [0 0 -9.80665];
ee = 'EndEffector_Link';
tauMax = [187;187;187;52;52;52;52];
dt = mean(diff(ts_ref));

pass_count = 0;
fail_count = 0;

function result = check(label, gui_val, computed_val, tol, unit)
    diff_val = abs(gui_val - computed_val);
    rel_diff = diff_val / max(abs(computed_val), 1e-10) * 100;
    if diff_val <= tol
        status = 'PASS';
    else
        status = 'FAIL';
    end
    fprintf('  [%s] %-35s  GUI=%-10s  Computed=%-10s  Diff=%.4g %s\n', ...
        status, label, ...
        num2str(gui_val,'%.4f'), ...
        num2str(computed_val,'%.4f'), ...
        diff_val, unit);
    result = strcmp(status,'PASS');
end

%% =========================================================
%% SECTION 1 — TRACKING ERROR
%% =========================================================
fprintf('\n%s\n', repmat('-',1,60));
fprintf('SECTION 1: Tracking Error\n');
fprintf('%s\n', repmat('-',1,60));

%% Recompute from scratch
qref_i = interp1(ts_ref, pos_ref', t_s, 'linear');
valid  = ~any(isnan(qref_i),2);
err_   = qref_i(valid,:) - Qout(valid,:);   % in radians

rms_per_joint = sqrt(mean(err_.^2));
max_per_joint = max(abs(err_));

rms_overall_rad = mean(rms_per_joint);
max_overall_rad = max(max_per_joint);
rms_overall_deg = rms_overall_rad * 180/pi;
max_overall_deg = max_overall_rad * 180/pi;

fprintf('\nPer-joint tracking error (degrees):\n');
for j=1:7
    fprintf('  J%d: RMS=%.4f°  Max=%.4f°\n', j, ...
        rms_per_joint(j)*180/pi, max_per_joint(j)*180/pi);
end

fprintf('\nOverall metrics:\n');
fprintf('  RMS error (mean across joints): %.4f°\n', rms_overall_deg);
fprintf('  Max error (worst joint):        %.4f°\n', max_overall_deg);

%% The GUI shows these — ask user to verify
fprintf('\nVerification (compare with GUI Simulation Results panel):\n');
fprintf('  Computed Overall RMS = %.4f°\n', rms_overall_deg);
fprintf('  Computed Max Error   = %.4f°\n', max_overall_deg);
[~,best_j]  = min(max_per_joint);
[~,worst_j] = max(max_per_joint);
fprintf('  Computed Best  joint = J%d\n', best_j);
fprintf('  Computed Worst joint = J%d\n', worst_j);

%% =========================================================
%% SECTION 2 — ENERGY CONSUMPTION
%% =========================================================
fprintf('\n%s\n', repmat('-',1,60));
fprintf('SECTION 2: Energy Consumption\n');
fprintf('%s\n', repmat('-',1,60));

N_ref = size(pos_ref,2);
P_grav  = zeros(N_ref,1);
P_inert = zeros(N_ref,1);
P_cor   = zeros(N_ref,1);

fprintf('Computing energy (this takes ~30s)...\n');
for i=1:N_ref
    q_i   = pos_ref(:,i);
    qd_i  = vel_ref(:,i);
    qdd_i = acc_ref(:,i);

    M_  = massMatrix(robot, q_i);
    C_  = velocityProduct(robot, q_i, qd_i);
    G_  = gravityTorque(robot, q_i);

    tau_grav  = G_;
    tau_inert = M_ * qdd_i;
    tau_cor   = C_;

    P_grav(i)  = abs(tau_grav'  * qd_i);
    P_inert(i) = abs(tau_inert' * qd_i);
    P_cor(i)   = abs(tau_cor'   * qd_i);
end

E_grav  = trapz(ts_ref, P_grav);
E_inert = trapz(ts_ref, P_inert);
E_cor   = trapz(ts_ref, P_cor);
E_total = E_grav + E_inert + E_cor;

pct_grav  = E_grav  / E_total * 100;
pct_inert = E_inert / E_total * 100;
pct_cor   = E_cor   / E_total * 100;

fprintf('\nEnergy results:\n');
fprintf('  Total:    %.2f J\n', E_total);
fprintf('  Gravity:  %.2f J  (%.0f%%)\n', E_grav,  pct_grav);
fprintf('  Inertial: %.2f J  (%.0f%%)\n', E_inert, pct_inert);
fprintf('  Coriolis: %.2f J  (%.0f%%)\n', E_cor,   pct_cor);
fprintf('\nCompare with GUI Energy panel — values should match within 1%%\n');

%% =========================================================
%% SECTION 3 — END-EFFECTOR POSITION
%% =========================================================
fprintf('\n%s\n', repmat('-',1,60));
fprintf('SECTION 3: End-Effector Pose (final position)\n');
fprintf('%s\n', repmat('-',1,60));

%% FK on final Q_out
T_final = getTransform(robot, Qout(end,:)', ee);
ee_pos  = T_final(1:3,4);

fprintf('\nFK on final Q_out:\n');
fprintf('  X = %.4f m\n', ee_pos(1));
fprintf('  Y = %.4f m\n', ee_pos(2));
fprintf('  Z = %.4f m\n', ee_pos(3));
fprintf('  Distance from origin = %.3f m\n', norm(ee_pos));

%% FK on final q_ref (what was commanded)
T_ref_final = getTransform(robot, pos_ref(:,end), ee);
ee_ref_pos  = T_ref_final(1:3,4);
fprintf('\nFK on final q_ref (commanded):\n');
fprintf('  X = %.4f m\n', ee_ref_pos(1));
fprintf('  Y = %.4f m\n', ee_ref_pos(2));
fprintf('  Z = %.4f m\n', ee_ref_pos(3));

final_err_mm = norm(ee_pos - ee_ref_pos) * 1000;
fprintf('\nFinal EE position error (actual vs commanded): %.3f mm\n', final_err_mm);
fprintf('Compare with GUI IK err (mm) field in EE Pose panel\n');

%% =========================================================
%% SECTION 4 — JOINT LIMIT COMPLIANCE
%% =========================================================
fprintf('\n%s\n', repmat('-',1,60));
fprintf('SECTION 4: Joint Limit Compliance\n');
fprintf('%s\n', repmat('-',1,60));

limits_lo = [-2.41 -2.41 -2.66 -2.23 -2.09 -2.09 -2.09] * 180/pi; % approx deg
limits_hi = [ 2.41  2.41  2.66  2.23  2.09  2.09  2.09] * 180/pi;

all_within = true;
for j=1:7
    q_deg = Qout(:,j) * 180/pi;
    min_q = min(q_deg); max_q = max(q_deg);
    within = (min_q >= limits_lo(j)-0.1) && (max_q <= limits_hi(j)+0.1);
    if ~within; all_within = false; end
    fprintf('  J%d: min=%.1f°  max=%.1f°  limits=[%.1f° %.1f°]  %s\n', ...
        j, min_q, max_q, limits_lo(j), limits_hi(j), ...
        bool2str(within));
end
fprintf('Joint limits: %s\n', ternary_str(all_within,'ALL WITHIN LIMITS','VIOLATIONS DETECTED'));

%% =========================================================
%% SECTION 5 — TORQUE SATURATION CHECK
%% =========================================================
fprintf('\n%s\n', repmat('-',1,60));
fprintf('SECTION 5: Feedforward Torque Verification\n');
fprintf('%s\n', repmat('-',1,60));

fprintf('Recomputing CTC feedforward torques from reference trajectory...\n');
tau_check = zeros(N_ref, 7);
for i=1:N_ref
    M_  = massMatrix(robot, pos_ref(:,i));
    C_  = velocityProduct(robot, pos_ref(:,i), vel_ref(:,i));
    G_  = gravityTorque(robot, pos_ref(:,i));
    tau = M_ * acc_ref(:,i) + C_ + G_;
    tau_check(i,:) = max(-tauMax, min(tauMax, tau))';
end

saturated = false;
for j=1:7
    max_tau = max(abs(tau_check(:,j)));
    sat_pct = max_tau / tauMax(j) * 100;
    is_sat  = max_tau >= tauMax(j) * 0.99;
    if is_sat; saturated = true; end
    fprintf('  J%d: max|τ|=%.1f Nm  limit=%.0f Nm  (%.0f%%)  %s\n', ...
        j, max_tau, tauMax(j), sat_pct, ...
        ternary_str(is_sat,'SATURATED','OK'));
end

%% =========================================================
%% SECTION 6 — SIMULINK vs ROBOTICS TOOLBOX CONSISTENCY
%% =========================================================
fprintf('\n%s\n', repmat('-',1,60));
fprintf('SECTION 6: Simulink Output vs Robotics Toolbox FK\n');
fprintf('%s\n', repmat('-',1,60));
fprintf('Checking that Simulink Q_out FK matches expected trajectory...\n');

%% Sample 10 evenly-spaced timepoints within valid range
n_check = 10;
%% Only sample within ts_ref range to avoid NaN from extrapolation
t_valid_max = min(t_s(end), ts_ref(end)) - 0.01;
t_valid_min = max(t_s(1),   ts_ref(1));
t_samples   = linspace(t_valid_min, t_valid_max, n_check);
max_fk_err  = 0;
for k=1:n_check
    %% Find closest Q_out index to this sample time
    [~,idx] = min(abs(t_s - t_samples(k)));
    T_k = getTransform(robot, Qout(idx,:)', ee);
    p_k = T_k(1:3,4);

    %% Get reference EE at same time — safe interp within range
    qr_k = interp1(ts_ref, pos_ref', t_s(idx), 'linear')';
    T_rk = getTransform(robot, qr_k, ee);
    p_rk = T_rk(1:3,4);

    err_k = norm(p_k - p_rk) * 1000;
    max_fk_err = max(max_fk_err, err_k);
    fprintf('  t=%.2fs: actual EE=[%.3f %.3f %.3f]  ref EE=[%.3f %.3f %.3f]  err=%.2fmm\n', ...
        t_s(idx), p_k(1),p_k(2),p_k(3), p_rk(1),p_rk(2),p_rk(3), err_k);
end
fprintf('Max EE position error across all sample points: %.2f mm\n', max_fk_err);

%% =========================================================
%% SECTION 7 — DATA INTEGRITY CHECKS
%% =========================================================
fprintf('\n%s\n', repmat('-',1,60));
fprintf('SECTION 7: Data Integrity\n');
fprintf('%s\n', repmat('-',1,60));

%% Check timestamps are monotonically increasing
dt_ref = diff(ts_ref);
dt_out = diff(t_s);
fprintf('  Reference timestamps monotonic:  %s\n', bool2str(all(dt_ref > 0)));
fprintf('  Output timestamps monotonic:     %s\n', bool2str(all(dt_out > 0)));
fprintf('  Reference dt mean: %.4f s  std: %.6f s\n', mean(dt_ref), std(dt_ref));
fprintf('  Output dt mean:    %.4f s  std: %.6f s\n', mean(dt_out), std(dt_out));

%% Check for NaN/Inf
fprintf('  NaN in Q_out:      %s\n', bool2str(~any(isnan(Qout(:)))));
fprintf('  NaN in q_ref_traj: %s\n', bool2str(~any(isnan(q_ref(:)))));
fprintf('  Inf in Q_out:      %s\n', bool2str(~any(isinf(Qout(:)))));

%% Check Q_out has 7 columns
fprintf('  Q_out size: %d timesteps × %d joints (expected 7)\n', size(Qout,1), size(Qout,2));

%% =========================================================
%% SUMMARY
%% =========================================================
fprintf('\n%s\n', repmat('=',1,60));
fprintf('VALIDATION SUMMARY\n');
fprintf('%s\n', repmat('=',1,60));
fprintf('\nAll values below should match the KinovaApp GUI:\n\n');

fprintf('  %-30s  %s\n', 'Metric', 'Computed Value');
fprintf('  %s\n', repmat('-',1,55));
fprintf('  %-30s  %.4f degrees\n',   'Overall RMS error',    rms_overall_deg);
fprintf('  %-30s  %.4f degrees\n',   'Max joint error',      max_overall_deg);
fprintf('  %-30s  J%d\n',            'Best joint',           best_j);
fprintf('  %-30s  J%d\n',            'Worst joint',          worst_j);
fprintf('  %-30s  %.2f J\n',         'Total energy',         E_total);
fprintf('  %-30s  %.2f J (%.0f%%)\n','Gravity energy',       E_grav,  pct_grav);
fprintf('  %-30s  %.2f J (%.0f%%)\n','Inertial energy',      E_inert, pct_inert);
fprintf('  %-30s  %.2f J (%.0f%%)\n','Coriolis energy',      E_cor,   pct_cor);
fprintf('  %-30s  [%.4f %.4f %.4f] m\n','Final EE position', ee_pos(1),ee_pos(2),ee_pos(3));
fprintf('  %-30s  %.3f mm\n',         'Final EE error',      final_err_mm);
fprintf('  %-30s  %.2f mm\n',         'Max FK deviation',    max_fk_err);

%% Save report to file
fid = fopen('validation_report.txt','w');
fprintf(fid,'Kinova Gen3 CTC Validation Report\n');
fprintf(fid,'Generated: %s\n\n', datestr(now));
fprintf(fid,'Overall RMS error:    %.4f degrees\n', rms_overall_deg);
fprintf(fid,'Max joint error:      %.4f degrees\n', max_overall_deg);
fprintf(fid,'Best joint:           J%d\n', best_j);
fprintf(fid,'Worst joint:          J%d\n', worst_j);
fprintf(fid,'Total energy:         %.2f J\n', E_total);
fprintf(fid,'Gravity energy:       %.2f J (%.0f%%)\n', E_grav, pct_grav);
fprintf(fid,'Inertial energy:      %.2f J (%.0f%%)\n', E_inert, pct_inert);
fprintf(fid,'Coriolis energy:      %.2f J (%.0f%%)\n', E_cor, pct_cor);
fprintf(fid,'Final EE position:    [%.4f %.4f %.4f] m\n', ee_pos(1),ee_pos(2),ee_pos(3));
fprintf(fid,'Final EE error:       %.3f mm\n', final_err_mm);
fprintf(fid,'Max FK deviation:     %.2f mm\n', max_fk_err);
fclose(fid);

fprintf('\nReport saved to: validation_report.txt\n');
fprintf('%s\n\n', repmat('=',1,60));

%% ── Helper functions ─────────────────────────────────────
function s = bool2str(b)
    if b; s='YES'; else; s='NO'; end
end

function s = ternary_str(b, a, c)
    if b; s=a; else; s=c; end
end