function commands = cav_control_policy(vehicles, lanePhaseMap, phaseIdxSeq, durSeq, idm, cav, vehHeadway, cavHeadway, dt, tnow, yellowDur)
% CAV control policy with signal awareness and CBF-based rear-end safety.
% Returns an array of deferred TraCI commands (vid, v_next, laneID, lanePos_next, x/y_target).
% The caller is responsible for applying these commands via TraCI.
%
% yellowDur — clearance (yellow) shown after phase slots. Either a scalar
%             (every slot of durSeq ends with a yellow tail of this length)
%             or a vector with one entry per slot of durSeq giving that
%             slot's yellow tail (0 for slots without one). durSeq slots are
%             assumed to INCLUDE their yellow tail so that wall-clock timing
%             is correct. A lane is NOT green during the yellow tail.
%             For the extrapolated cycle beyond the horizon, max(yellowDur)
%             is used after every phase.

% ---- Rear-end safety constraint parameters (paper Eqs. 13, 18) ----
% h = p_j - p_i - s0 - v_i*T  (bumper-to-bumper gap minus standstill distance
% minus time-headway spacing), enforced through  h_dot + kappa*h >= 0.
% T and kappa are taken from the cav struct if present.
if isfield(cav, 'T_safe'),  cbf_T = cav.T_safe;  else, cbf_T = 0.5;  end
if isfield(cav, 'kappa'),   cbf_kappa = cav.kappa; else, cbf_kappa = 0.3; end
cbf_gamma = 0;         % unused (kept so the call signature is unchanged)
cbf_tol = 1e-6;        % CBF tolerance

nVeh = numel(vehicles);

if nVeh == 0
    commands = struct('vid', {}, 'v_next', {}, 'laneID', {}, 'lanePos_next', {}, 'x_target', {}, 'y_target', {}, 'u', {}, 'v', {}, 'distToStop', {}, 'diag', {});
    return;
end

% Pre-allocate command output to max possible size
emptyDiag = struct('t_target', NaN, 'u_nom', NaN, 'u_after_cbf', NaN, 'cbf_active', false, 'h', NaN, 'gap', NaN, ...
    'leader', '', 'v_leader', NaN, 'mustStop', false, 'stopActive', false, 'greenNow', false, 'greenEnd', NaN, ...
    'profile', '', 'standingStart', false, 'lineClamp', false, 'replanned', false);
commands = repmat(struct('vid', '', 'v_next', 0, 'laneID', '', 'lanePos_next', 0, 'x_target', 0, 'y_target', 0, 'u', 0, 'v', 0, 'distToStop', 0, 'diag', emptyDiag), 1, nVeh);
nCmd = 0;

% Group vehicles by lane to determine leading/following relationships
lanes = {vehicles.lane};
[uniqueLanes, ~, ic] = unique(lanes);
laneGroups = accumarray(ic, (1:nVeh)', [numel(uniqueLanes), 1], @(x) {x'});

% Processing order: per lane, from the stop line backwards, so that each
% vehicle's leader has already been planned (CAV) or predicted (HDV).
procOrder = [];
for g = 1:numel(laneGroups)
    idxg = laneGroups{g};
    [~, og] = sort([vehicles(idxg).distToStop], 'ascend');
    procOrder = [procOrder, idxg(og)]; %#ok<AGROW>
end
% Crossing times (relative to now) of the vehicles processed so far in this
% step: planned targets of CAVs, closed-form predictions of HDVs.
crossPred = containers.Map('KeyType', 'char', 'ValueType', 'double');
prevOnLane = containers.Map('KeyType', 'char', 'ValueType', 'char');
useModelPred = ~(isfield(cav, 'leaderPred') && strcmpi(cav.leaderPred, 'mintime'));
if useModelPred
    [startupDelay, dischargeHw] = queue_release_cached(idm, dt, vehHeadway);
end

% Process each vehicle
for vi = procOrder
    veh = vehicles(vi);
    leadID_p = '';
    if isKey(prevOnLane, veh.lane), leadID_p = prevOnLane(veh.lane); end
    prevOnLane(veh.lane) = veh.id;

    % Only CAVs on incoming lanes within the control zone are controlled;
    % the others (HDVs) only get a crossing-time prediction for their followers.
    if ~isfield(veh, 'typeID') || ~strcmpi(veh.typeID, 'CAV') || ...
        ~isKey(lanePhaseMap, veh.lane) || veh.distToStop > 300
        if useModelPred && isKey(lanePhaseMap, veh.lane)
            tLead = NaN;
            if ~isempty(leadID_p) && isKey(crossPred, leadID_p), tLead = crossPred(leadID_p); end
            crossPred(veh.id) = predict_hdv_crossing(veh, tLead, idm, vehHeadway, startupDelay, dischargeHw, ...
                lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, dt);
        end
        continue;
    end

    % Get vehicle ID and current state
    vid = veh.id;

    v_cav = veh.speed;

    % Check if vehicle is still on an incoming lane


    % *** If vehicle has crossed, apply maximum acceleration; otherwise apply control ***

        % Vehicle is approaching: apply signal-aware intersection control
        [u, diag] = apply_intersection_control(vi, veh, vehicles, laneGroups, uniqueLanes, ...
                                       lanePhaseMap, phaseIdxSeq, durSeq, idm, cav, ...
                                       vehHeadway, cavHeadway, dt, tnow, cbf_gamma, cbf_kappa, cbf_tol, yellowDur, crossPred);
        if useModelPred && ~isnan(diag.t_target)
            crossPred(veh.id) = diag.t_target;   % planned crossing, read by the follower
        end

    % Compute next state and store command for deferred application
    v_next = max(v_cav + u * dt, 0);
    v_next = min(v_next, cav.v_max);
    displacement = v_cav * dt + 0.5 * u * dt^2;
    lanePos_next = veh.lanePos + displacement;

    % Stop-line protection on red (paper Eq. 14), respecting the actuator
    % bounds of Eq. (12). If the commanded step would reach the stop line
    % (minus a standoff, so discretisation never carries the front bumper
    % onto the internal lane) while the lane is not green, brake toward a
    % stop at the standoff point with u = -v^2/(2 d), bounded below by u_min.
    % No instantaneous stops: with SUMO's own checks disabled they would be
    % executed literally. If even u_min cannot stop the vehicle in time,
    % apply u_min and report it: the vehicle will cross on red, which is a
    % coordination failure to be seen.
    if isfield(cav, 'line_standoff'), s_line = cav.line_standoff; else, s_line = 1.0; end
    distToStop_clamp = max(veh.distToStop - s_line, 0);
    if displacement >= distToStop_clamp
        [~, yEnd_c, greenNow_c, inYel_c] = own_window(veh.lane, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur);
        Lj_c = 21; if isfield(cav, 'junction_length'), Lj_c = cav.junction_length; end
        entryOK = greenNow_c || (inYel_c && Lj_c / max(v_cav, 0.1) <= yEnd_c);
        if ~entryOK
            diag.lineClamp = true;
            if v_cav <= 1e-6
                u = min(u, 0);
            elseif distToStop_clamp > 1e-3
                a_req = v_cav^2 / (2 * distToStop_clamp);
                u = max(-a_req, cav.u_min);
                if a_req > abs(cav.u_min) + 1e-6
                    fprintf('[cav_control_policy] %s: cannot stop before stop line on red (needs %.2f m/s^2, u_min = %.2f) at d = %.2f m, v = %.2f m/s\n', ...
                        vid, a_req, cav.u_min, distToStop_clamp, v_cav);
                end
            else
                u = cav.u_min;
            end
            v_next = max(v_cav + u * dt, 0);
            v_next = min(v_next, cav.v_max);
            displacement = v_cav * dt + 0.5 * u * dt^2;
            lanePos_next = veh.lanePos + displacement;
        end
    end

    nCmd = nCmd + 1;
    commands(nCmd) = struct('vid', vid, 'v_next', v_next, ...
        'laneID', veh.lane, 'lanePos_next', lanePos_next, ...
        'x_target', veh.pos(1) + displacement, ...
        'y_target', veh.pos(2), ...
        'u', u, 'v', v_cav, 'distToStop', veh.distToStop, 'diag', diag);
end
commands = commands(1:nCmd);
end

function [u, diag] = apply_intersection_control(vi, veh, vehicles, laneGroups, uniqueLanes, ...
                                        lanePhaseMap, phaseIdxSeq, durSeq, idm, cav, ...
                                        ~, cavHeadway, dt, ~, cbf_gamma, cbf_kappa, cbf_tol, yellowDur, crossPred)
% Signal-aware intersection control with CBF rear-end safety
% Applied only to CAVs approaching the intersection
% diag: per-step explanation of the command (for auditing).
diag = struct('t_target', NaN, 'u_nom', NaN, 'u_after_cbf', NaN, 'cbf_active', false, 'h', NaN, 'gap', NaN, ...
    'leader', '', 'v_leader', NaN, 'mustStop', false, 'stopActive', false, 'greenNow', false, 'greenEnd', NaN, ...
    'profile', '', 'standingStart', false, 'lineClamp', false, 'replanned', false);

% Get vehicle state from pre-queried data
pos_cav = veh.pos(:);
v_cav = veh.speed;
lane_cav = veh.lane;

% Safe time headway T of the rear-end constraint (paper Eqs. 13, 18)
if isfield(cav, 'T_safe'), cbf_T = cav.T_safe; else, cbf_T = 0.5; end

% Find lane group for ordering
laneGroupIdx = [];
for g = 1:numel(uniqueLanes)
    if strcmp(uniqueLanes{g}, lane_cav)
        laneGroupIdx = g;
        break;
    end
end

if isempty(laneGroupIdx)
    u = 0;
    return;
end

idx = laneGroups{laneGroupIdx};

% Get distances to stop line and sort to find leader
dist = zeros(numel(idx), 1);
for k = 1:numel(idx)
    dist(k) = max(vehicles(idx(k)).distToStop, 0);
end
[~, orderLocal] = sort(dist, 'ascend');
ordered = idx(orderLocal);

% Find position of this vehicle in ordering
cav_pos_in_order = find(ordered == vi);
if isempty(cav_pos_in_order)
    u = 0;
    return;
end

% Uncertainty class of this CAV's crossing (paper Remark 1): deterministic
% when it has no leader or a CAV leader; uncertain behind an HDV. The
% dilemma-zone margin and the commit-time estimate depend on it.
hasLeader = cav_pos_in_order > 1;
uncertainLeader = false;
if hasLeader
    lt = vehicles(ordered(cav_pos_in_order - 1)).typeID;
    uncertainLeader = ~strcmpi(lt, 'CAV');
end
if isfield(cav, 'green_margin'), gm_unc = cav.green_margin; else, gm_unc = 1.0; end
if isfield(cav, 'green_margin_det'), gm_det = cav.green_margin_det; else, gm_det = 0.3; end
cavEff = cav;
if uncertainLeader, cavEff.green_margin = gm_unc; else, cavEff.green_margin = gm_det; end
% Crossing-speed estimate and junction length used to decide whether an
% entry during the clearance interval still clears the junction in time.
cavEff.v_est = max(v_cav, 0.1);
if ~isfield(cavEff, 'junction_length'), cavEff.junction_length = 21; end

% Determine target crossing time
if cav_pos_in_order == 1
    % No leader: aim for earliest feasible green
    distToStop = max(veh.distToStop, 0);
    
    % SPECIAL CASE: If vehicle is nearly stationary and signal is green, accelerate at max immediately
    % This ensures aggressive acceleration from stop at green light rather than smooth launch
    signal_is_green = lane_green_at_time(lane_cav, 0, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur);
    if v_cav < 0.5 && signal_is_green && distToStop >= 0
        % Use maximum acceleration for standing start at green - skip profile generation
        u_nom = cav.u_max;
        diag.standingStart = true;
        % Set t_target to very close (just a placeholder, won't be used for profile)
        t_target_rel = 0;
    else
        % Normal optimal control trajectory
        t_earliest_rel = cav_oc_min_time_to_cross(distToStop, v_cav, cav);

        if crossing_time_ok(lane_cav, t_earliest_rel, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, cavEff)
            t_target_rel = t_earliest_rel;
        else
            t_target_rel = find_next_green_start(lane_cav, t_earliest_rel, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowDur, cavEff);
        end
        u_nom = -1;  % Flag to compute from profile
    end
else
    % Has leader: aim for leader_crossing + headway
    leader_idx = ordered(cav_pos_in_order - 1);

    pos_leader = vehicles(leader_idx).pos;
    v_leader = vehicles(leader_idx).speed;

    if isempty(pos_leader)
        u = 0;
        return;
    end

    % Leader's crossing time: its planned target if it is a CAV, the
    % closed-form prediction (free-road IDM + headways + signal) if it is an
    % HDV; both were computed earlier in this step. Fallback (and switch
    % cav.leaderPred = 'mintime'): leader's minimum-time crossing, snapped.
    leadID_c = vehicles(leader_idx).id;
    if nargin >= 19 && ~isempty(crossPred) && isKey(crossPred, leadID_c) && ~isnan(crossPred(leadID_c))
        t_leader_cross_rel = crossPred(leadID_c);
    else
        distToStop_leader = max(vehicles(leader_idx).distToStop, 0);
        t_leader_earliest_rel = cav_oc_min_time_to_cross(distToStop_leader, v_leader, cav);
        if crossing_time_ok(lane_cav, t_leader_earliest_rel, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, cavEff)
            t_leader_cross_rel = t_leader_earliest_rel;
        else
            t_leader_cross_rel = find_next_green_start(lane_cav, t_leader_earliest_rel, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowDur, cavEff);
        end
    end

    % Target: leader crossing + headway based on the fastest possible
    t_earliest_rel = cav_oc_min_time_to_cross(veh.distToStop, v_cav, cav);

    t_target_rel = t_leader_cross_rel + cavHeadway;


    if t_target_rel < t_earliest_rel
        t_target_rel = t_earliest_rel;
    end


    % Verify target is in green (with margin); if not, shift to next green
    if ~crossing_time_ok(lane_cav, t_target_rel, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, cavEff)
        t_target_rel = find_next_green_start(lane_cav, t_target_rel, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowDur, cavEff);
    end
    u_nom = -1;  % Flag: compute from profile
end

% Generate reference trajectory profile and compute control (if not already set)
if u_nom == -1
    distToStop = max(veh.distToStop, 0);
    T_plan = max(t_target_rel, dt);
    profile = cav_control_profile(v_cav, distToStop, T_plan, cav);
    diag.profile = char(profile.type);

    % Compute nominal control input for this time step
    t_since_start = 0;
    u_nom = cav_control_u(t_since_start, profile, cav);

    % Fallbacks when no closed-form case of the profile applies:
    if isfield(cav, 'line_standoff'), s_fb_pre = cav.line_standoff; else, s_fb_pre = 1.0; end
    if profile.type == ""
        coastDist = v_cav * T_plan;
        if abs(coastDist - distToStop) < 1e-6 * max(distToStop, 1)
            u_nom = 0;                       % exactly cruising to the target
            diag.profile = 'cruise';
        elseif coastDist > distToStop && distToStop > s_fb_pre
            % Target is far in the future (next green): the unconstrained
            % linear profile would need v < 0. Decelerate smoothly toward a
            % stop at the standoff point instead of coasting and braking hard.
            % (Not within the standoff itself: there the vehicle is crossing.)
            u_nom = -v_cav^2 / (2 * max(distToStop - s_fb_pre, 0.5));
            diag.profile = 'stop-fallback';
        elseif coastDist > distToStop
            u_nom = 0;                       % within the standoff, crossing
            diag.profile = 'cruise';
        else
            % Near-cruise acceleration side: bounded linear profile.
            alpha_fb = 3 * (v_cav * T_plan - distToStop) / T_plan^3;
            u_nom = -alpha_fb * T_plan;
            if v_cav >= cav.v_max - 1e-6, u_nom = min(u_nom, 0); end
            diag.profile = 'linear-fallback';
        end
    end
end
diag.t_target = t_target_rel;

% Nominal control is subject to the actuation bounds (paper Eq. 12)
u_nom = min(max(u_nom, cav.u_min), cav.u_max);
% At rest at the standoff point on red: hold until green (the profile would
% otherwise creep the last metre toward the line and be braked again).
if isfield(cav, 'line_standoff'), s_hold = cav.line_standoff; else, s_hold = 1.0; end
if v_cav < 0.5 && max(veh.distToStop, 0) <= s_hold + 0.5 && ...
        ~lane_green_at_time(lane_cav, 0, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur)
    u_nom = min(u_nom, 0);
    diag.profile = 'hold-at-line';
end
diag.u_nom = u_nom;

% Get leader state for CBF constraint
if cav_pos_in_order > 1
    leader_idx = ordered(cav_pos_in_order - 1);
    pos_leader = vehicles(leader_idx).pos;
    gap_leader = norm(pos_leader(:) - pos_cav) - idm.L;   % bumper-to-bumper
    v_leader = vehicles(leader_idx).speed;
    a_leader = 0;
else
    % No leader on this lane. If SUMO sees a vehicle ahead beyond the stop
    % line (just crossed, possibly slow), use it for the rear-end constraint.
    if isfield(veh, 'extLeaderGap') && ~isnan(veh.extLeaderGap)
        gap_leader = veh.extLeaderGap;
        v_leader = veh.extLeaderSpeed;
        a_leader = 0;
        diag.leader = 'ext';
    else
        % CBF is non-constraining
        gap_leader = 1000;
        v_leader = 5;
        a_leader = 0;
    end
end

% Reactive safety tracking problem (paper Eq. 18):
%   min 1/2 (u - u_nom)^2  s.t.  u_min <= u <= u_max,  h_dot + kappa*h >= 0
% with h = gap - s0 - v_cav*T and h_dot = v_leader - v_cav - u*T.
% Barrier form (paper Eq. 13; 'braking' = input-constrained form):
%   headway: h = gap - s0 - v*T
%   braking: h = gap - s0 - v*T - max(v^2 - v_l^2, 0)/(2|u_min|)
% h_dot stays linear in u with T_eff = T + v/|u_min| (when v > v_l).
[h, cExtra] = barrier_terms(gap_leader, v_cav, v_leader, idm.s0, cbf_T, cav);
[u, cbf_active] = cbf_project(u_nom, h, v_leader - v_cav, cbf_T + cExtra, cbf_kappa, cav.u_min, cav.u_max, cbf_tol, dt);
diag.h = h; diag.gap = gap_leader; diag.v_leader = v_leader; diag.cbf_active = cbf_active; diag.u_after_cbf = u;
if cav_pos_in_order > 1, diag.leader = vehicles(ordered(cav_pos_in_order - 1)).id; end

% If CBF is active, check feasibility of reaching target crossing time
if cbf_active
    % If CBF forced deceleration, re-estimate the earliest crossing from the
    % current state (min-time plan the CAV tracks). The original code used
    % distToStop / v_cav, i.e. crawling at the current speed all the way to
    % the line, which abandons a green as soon as the barrier has slowed the
    % CAV behind a queued leader that is about to launch.
    if u < u_nom && v_cav > 0
        t_actual_est = max(t_target_rel, cav_oc_min_time_to_cross(distToStop, v_cav, cav));

        % Check if estimated crossing is still in target green window
        if ~crossing_time_ok(lane_cav, t_actual_est, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, cavEff)
            % Crossing has shifted out of green; replan to next green
            t_target_next = find_next_green_start(lane_cav, t_actual_est, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowDur, cavEff);
            t_target_rel = t_target_next;   % keep the plan consistent for the stop-line check below
            diag.replanned = true; diag.t_target = t_target_rel;
            T_plan_next = max(t_target_next, dt);
            profile = cav_control_profile(v_cav, distToStop, T_plan_next, cav);
            u = cav_control_u(t_since_start, profile, cav);

            % Re-apply CBF to new nominal control
            u = min(max(u, cav.u_min), cav.u_max);
            [u, ~] = cbf_project(u, h, v_leader - v_cav, cbf_T + cExtra, cbf_kappa, cav.u_min, cav.u_max, cbf_tol, dt);
        end
    end
end

% =========================================================
% RED-LIGHT STOP-LINE SAFETY CONSTRAINT
% =========================================================
% Regardless of trajectory plan or CBF, ensure the vehicle can stop
% before the stop line whenever the signal is not green.  This is the
% final safety layer that prevents stop-line overshoot during red.
%
% Forward-invariance condition:
%   (v + u·dt)² / (2·b)  ≤  d − v·dt − ½·u·dt²
% i.e. after applying u for one dt the stopping distance must not
% exceed the remaining distance to the stop line.
distToStop_sl = max(veh.distToStop, 0);
if isfield(cav, 'line_standoff'), s_line = cav.line_standoff; else, s_line = 1.0; end
distToStop_eff = max(distToStop_sl - s_line, 0);   % stop this far before the line
if distToStop_sl > 0 && v_cav > 0
    [gEnd, yEnd, greenNow, inOwnYellow] = own_window(lane_cav, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur);
    greenEnd = gEnd;
    % Latest admissible entry time: the end of the contiguous green, extended
    % into the clearance interval by the part of it in which the vehicle,
    % crossing at (at least) its current speed, still clears the junction
    % before the conflicting green starts.
    latestEntry = max(gEnd, yEnd - cavEff.junction_length / cavEff.v_est);
    % The vehicle must remain able to stop before the line unless it will
    % certainly enter before latestEntry:
    %  - light green now (or own clearance running), AND
    %  - its commit time lies at least green_margin before latestEntry.
    %    Deterministic case (no leader / CAV leader, paper Remark 1): the
    %    commit time is the min-time crossing the CAV's own plan tracks, with
    %    the small margin green_margin_det. Uncertain case (HDV leader): a
    %    cautious estimate accelerating at u_max*(1 - v/v_max) (full from
    %    standstill, none at cruise) with the larger margin green_margin.
    % While mustStop is true the vehicle is only kept STOPPABLE (stopping
    % distance <= distance to the standoff point); far from the line this
    % does not restrict u at all.
    gm = cavEff.green_margin;
    if uncertainLeader
        t_at_current_speed = commit_time_to_cross(distToStop_sl, v_cav, cav);
    else
        t_at_current_speed = cav_oc_min_time_to_cross(distToStop_sl, v_cav, cav);
    end
    mustStop = ~(greenNow || inOwnYellow) || (t_at_current_speed > latestEntry - gm) || (t_target_rel > latestEntry - gm);
    diag.greenNow = greenNow; diag.greenEnd = latestEntry; diag.mustStop = mustStop;
    if mustStop
        b = abs(cav.u_min);
        % Quadratic in u:  dt²·u² + dt(2v+b·dt)·u + (v²+2bv·dt−2b·d) ≤ 0
        A_q = dt^2;
        B_q = dt * (2*v_cav + b*dt);
        C_q = v_cav^2 + 2*b*v_cav*dt - 2*b*distToStop_eff;
        disc_q = B_q^2 - 4*A_q*C_q;
        if disc_q >= 0
            u_safe = (-B_q + sqrt(disc_q)) / (2*A_q);
            diag.stopActive = u > u_safe + 1e-9;
            u = max(min(u, u_safe), cav.u_min);
        else
            diag.stopActive = true;
            u = cav.u_min;
        end
    end
end
end

function t = predict_hdv_crossing(veh, tLead, idm, hf, startupDelay, dischargeHw, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, dt)
% Closed-form crossing-time prediction of an HDV (same model as the signal
% optimizer): free-road IDM arrival, coupled to the predicted crossing of its
% leader by the saturation headway hf; if the signal blocks it, it crosses at
% the next green start plus the IDM start-up delay, and at least one
% discharge headway after its leader. No car-following is simulated: vehicles
% interact only through the headways, so a lane is predicted in O(n).
d = max(veh.distToStop, 0);
v = max(veh.speed, 0);
tReady = hdv_idm_free_crossing_time(d, v, idm);
if ~isnan(tLead), tReady = max(tReady, tLead + hf); end
if lane_green_at_time(veh.lane, tReady, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur)
    t = tReady;
else
    tg = next_green_plain(veh.lane, tReady, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, dt);
    t = tg + startupDelay;
    if ~isnan(tLead), t = max(t, tLead + dischargeHw); end
end
end

function tg = next_green_plain(laneID, t0, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, dt)
% Start of the next green of this lane at or after t0 (no margin). Analytic
% walk over the horizon slots; stepping search beyond the horizon.
if ~isKey(lanePhaseMap, laneID), tg = t0; return; end
gv = lanePhaseMap(laneID);
acc = 0;
for i = 1:numel(durSeq)
    if durSeq(i) <= 1e-9, continue; end
    s0 = acc; acc = acc + durSeq(i);
    ge = s0 + max(durSeq(i) - yellow_of_slot(yellowDur, i), 0);
    if gv(phaseIdxSeq(i)) && ge > s0 && ge > t0
        tg = max(s0, t0);
        return;
    end
end
tg = find_next_green_start(laneID, max(t0, acc), phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowDur, struct('green_margin', 0));
end

function [startupDelay, dischargeHw] = queue_release_cached(idm, dt, hf)
% IDM start-up delay and discharge headway from a two-vehicle launch,
% computed once per parameter set and cached.
persistent cacheKey cacheVal
key = sprintf('%g_', idm.a, idm.b, idm.T, idm.s0, idm.v0, idm.delta, idm.L, dt, hf);
if isempty(cacheKey) || ~strcmp(cacheKey, key)
    [sd, qh] = idm_queue_release_characteristics(idm, dt, hf);
    cacheKey = key; cacheVal = [sd, qh];
end
startupDelay = cacheVal(1); dischargeHw = cacheVal(2);
end

function tCross = hdv_idm_free_crossing_time(distToStop, v0, idm, ~)
% Analytical free-road IDM crossing time (no leader, no signal); same as in
% optimize_signal_plan.m. Constant acceleration at the initial IDM free-road
% rate until the desired speed, then cruise.
if distToStop <= 0
    tCross = 0;
    return;
end
v = max(v0, 0);
a0 = idm.a * (1 - (v * idm.inv_v0)^idm.delta);
if a0 < 1e-6
    tCross = distToStop / max(v, 0.01);
    return;
end
t_acc = (idm.v0 - v) / a0;
d_acc = v * t_acc + 0.5 * a0 * t_acc^2;
if d_acc >= distToStop
    disc = v^2 + 2 * a0 * distToStop;
    tCross = (-v + sqrt(max(disc, 0))) / a0;
else
    tCross = t_acc + (distToStop - d_acc) / idm.v0;
end
end

function a = idm_free_accel(v, idm)
freeTerm = (v * idm.inv_v0) .^ idm.delta;
a = idm.a * (1 - freeTerm);
end

function a = idm_accel(v, vLead, p, pLead, idm)
if isfinite(pLead)
    gap = pLead - p - idm.L;
else
    gap = inf;
end
if gap < 0.1
    gap = 0.1;
end
closing = v - vLead;
sStar = idm.s0 + max(0, v * idm.T + (v * closing) / (2 * idm.ab));
freeTerm = (v * idm.inv_v0).^idm.delta;
if isfinite(gap)
    interactTerm = (sStar / gap)^2;
else
    interactTerm = 0;
end
a = idm.a * (1 - freeTerm - interactTerm);
end

function [startupDelay, queueHeadway] = idm_queue_release_characteristics(idm, dt, minHeadway)
% Queue release parameters from a two-vehicle IDM launch (same as in
% optimize_signal_plan.m): startupDelay = first queued vehicle's time to
% cross after green; queueHeadway = gap between first and second crossings.
leadLength = max(idm.L, 0.1);
xLead = -leadLength; vLead = 0;
xFollow = xLead - (idm.L + idm.s0); vFollow = 0;
t = 0; tMax = 120; nSteps = max(2, ceil(tMax / dt));
tLeadCross = NaN; tFollowCross = NaN;
for k = 1:nSteps
    aLead = idm_free_accel(vLead, idm);
    vLeadNext = max(vLead + aLead * dt, 0);
    xLeadNext = xLead + vLead * dt + 0.5 * aLead * dt^2;
    aFollow = idm_accel(vFollow, vLead, xFollow, xLead, idm);
    vFollowNext = max(vFollow + aFollow * dt, 0);
    xFollowNext = xFollow + vFollow * dt + 0.5 * aFollow * dt^2;
    if isnan(tLeadCross) && xLeadNext >= 0
        frac = min(max((0 - xLead) / max(xLeadNext - xLead, eps), 0), 1);
        tLeadCross = t + frac * dt;
    end
    if isnan(tFollowCross) && xFollowNext >= 0
        frac = min(max((0 - xFollow) / max(xFollowNext - xFollow, eps), 0), 1);
        tFollowCross = t + frac * dt;
    end
    xLead = xLeadNext; vLead = vLeadNext; xFollow = xFollowNext; vFollow = vFollowNext;
    t = t + dt;
    if ~isnan(tLeadCross) && ~isnan(tFollowCross), break; end
end
if isnan(tLeadCross), tLeadCross = max(dt, 1.0); end
if isnan(tFollowCross), tFollowCross = tLeadCross + max(idm.T, dt); end
startupDelay = max(tLeadCross, dt);
queueHeadway = max([tFollowCross - tLeadCross, minHeadway, dt]);
end

function profile = cav_control_profile(v0, l, T, limits)
% Build control profile for given terminal time T and distance l.
% Returns structure with .type, .alpha, .u0, .tau1, .tau2, .T
% Types: "linear", "flag0", "flag1", "flag2"
%
% Based on OC.m optimal control analysis:
% - Linear: simple acceleration/deceleration with linear profile
% - Flag0: accelerate with ramp-down
% - Flag1: accelerate at max, then ramp down
% - Flag2: accelerate at max, decelerate, hold pattern

profile.type = "";
profile.alpha = 0;
profile.u0 = 0;
profile.tau1 = 0;
profile.tau2 = 0;
profile.T = T;

if T <= 0 || l <= 0
    return
end

% Check if target time is feasible
T_min = cav_oc_min_time_to_cross(l, v0, limits);
if T_min > T
    T = T_min;
    profile.T = T;
end

% Compute alpha for linear profile
alpha = 3 * (v0 * T - l) / (T^3);
coast_distance = v0 * T;

% ===== CASE 1: Deceleration side (coast_distance > l) =====
if coast_distance > l
    u0 = alpha * (0 - T);
    v_final = v0 + 0.5 * alpha * T^2 - alpha * T^2;
    if u0 > limits.u_min && v_final >= limits.v_min
        profile.type = "linear";
        profile.alpha = alpha;
        profile.u0 = u0;
        profile.tau1 = 0;
        profile.tau2 = 0;
    end
    return
end

% ===== CASE 2: Acceleration side (coast_distance < l) =====
if coast_distance < l
    u0 = alpha * (0 - T);
    v_final = v0 + 0.5 * alpha * T^2 - alpha * T^2;

    % ===== CASE 2a: Simple linear (no constraints active) =====
    if u0 <= limits.u_max && v_final <= limits.v_max
        profile.type = "linear";
        profile.alpha = alpha;
        profile.u0 = u0;
        profile.tau1 = 0;
        profile.tau2 = 0;
        return
    end

    % ===== CASE 2b: u0 <= u_max AND v_final > v_max (speed exceeds limit) =====
    if u0 <= limits.u_max && v_final > limits.v_max
        den = v0 - limits.v_max;
        if abs(den) < 1e-9
            tau_1 = T - sqrt(3) * sqrt(max(T^2 - (2 / limits.u_max) * (l - v0 * T), 0));
            profile.type = "flag1";
            profile.alpha = limits.u_max / (tau_1 - T);
            profile.u0 = limits.u_max;
            profile.tau1 = tau_1;
            profile.tau2 = 0;
            return
        end

        tau1 = (3 * (l - limits.v_max * T)) / den;
        u0_adj = (2 * (limits.v_max - v0) / tau1);

        if u0_adj < limits.u_max
            profile.type = "flag0";
            profile.alpha = -u0_adj / tau1;
            profile.u0 = u0_adj;
            profile.tau1 = tau1;
            profile.tau2 = 0;
            return
        end

        tau_1 = T - sqrt(3) * sqrt(max(T^2 - (2 / limits.u_max) * (l - v0 * T), 0));
        if v0 + (limits.u_max / 2) * (T + tau_1) <= limits.v_max
            profile.type = "flag1";
            profile.alpha = limits.u_max / (tau_1 - T);
            profile.u0 = limits.u_max;
            profile.tau1 = tau_1;
            profile.tau2 = 0;
            return
        end

        qa = (limits.u_max / 2);
        qb = (v0 - limits.v_max);
        qg = 3 * (l - T * limits.v_max) + 2 * (v0 - limits.v_max)^2 / limits.u_max;
        coeffs = [qa qb qg];
        solutions = roots(coeffs);

        real_solutions = solutions(abs(imag(solutions)) < 1e-9);
        real_solutions = real(real_solutions);
        valid_idx = real_solutions >= (limits.v_max - v0) / limits.u_max & real_solutions <= T;
        valid = sort(real_solutions(valid_idx));
        if ~isempty(valid)
            tau2 = valid(1);
            tau1_f2 = 2 * (limits.v_max - v0) / limits.u_max - tau2;
            profile.type = "flag2";
            profile.alpha = -limits.u_max / (tau2 - tau1_f2);
            profile.u0 = limits.u_max;
            profile.tau1 = tau1_f2;
            profile.tau2 = tau2;
            return
        end
    end

    % ===== CASE 2c: u0 > u_max (need higher accel than available) =====
    if u0 > limits.u_max
        tau1 = T - sqrt(3) * sqrt(max(T^2 - (2 / limits.u_max) * (l - v0 * T), 0));

        if v0 + (limits.u_max / 2) * (T + tau1) <= limits.v_max
            profile.type = "flag1";
            profile.alpha = limits.u_max / (tau1 - T);
            profile.u0 = limits.u_max;
            profile.tau1 = tau1;
            profile.tau2 = 0;
            return
        end

        qa = (limits.u_max / 2);
        qb = (v0 - limits.v_max);
        qg = 3 * (l - T * limits.v_max) + 2 * (v0 - limits.v_max)^2 / limits.u_max;
        coeffs = [qa qb qg];
        solutions = roots(coeffs);

        real_solutions = solutions(abs(imag(solutions)) < 1e-9);
        real_solutions = real(real_solutions);
        valid_idx = real_solutions >= (limits.v_max - v0) / limits.u_max & real_solutions <= T;
        valid = sort(real_solutions(valid_idx));
        if ~isempty(valid)
            tau2 = valid(1);
            tau1_f2 = 2 * (limits.v_max - v0) / limits.u_max - tau2;
            profile.type = "flag2";
            profile.alpha = -limits.u_max / (tau2 - tau1_f2);
            profile.u0 = limits.u_max;
            profile.tau1 = tau1_f2;
            profile.tau2 = tau2;
            return
        end
    end
end
end

function u = cav_control_u(t, profile, limits)
% Evaluate control input at time t for a profile.
u = 0;
if profile.type == "linear"
    u = profile.alpha * (t - profile.T);
elseif profile.type == "flag0"
    u = control_input_vec(profile.u0, profile.tau1, [], t, 0, profile.T, limits.u_max);
elseif profile.type == "flag1"
    u = control_input_vec(profile.u0, profile.tau1, [], t, 1, profile.T, limits.u_max);
elseif profile.type == "flag2"
    u = control_input_vec(profile.u0, profile.tau1, profile.tau2, t, 2, profile.T, limits.u_max);
end
end

function u = control_input_vec(u0, tau1, tau2, t, flag, T, u_max)
% Vectorized piecewise control profile matching OC policy cases.
u = zeros(size(t));

if flag == 0
    if tau1 <= 0
        return;
    end
    idx1 = t <= tau1;
    u(idx1) = u0 * (1 - t(idx1) / tau1);
elseif flag == 1
    idx1 = t <= tau1;
    idx2 = t > tau1 & t <= T;
    u(idx1) = u_max;
    den = (tau1 - T);
    if abs(den) > eps
        u(idx2) = u_max / den * (t(idx2) - T);
    end
else
    idx1 = t <= tau1;
    idx2 = t > tau1 & t <= tau2;
    u(idx1) = u_max;
    den = (tau2 - tau1);
    if abs(den) > eps
        u(idx2) = (-u_max / den) * (t(idx2) - tau2);
    end
end
end

function T_min = cav_oc_min_time_to_cross(l, v0, cav)
% Minimum-time arrival for the CAV based on optimal control.
if l <= 0
    T_min = 0;
    return
end

v0 = max(v0, cav.v_min);

t1 = (cav.v_max - v0) / cav.u_max;
t1 = max(t1, 0);

s1 = v0 * t1 + 0.5 * cav.u_max * t1^2;

if s1 >= l
    disc = v0^2 + 2 * cav.u_max * l;
    disc = max(disc, 0);
    T_min = (-v0 + sqrt(disc)) / cav.u_max;
else
    T_min = t1 + (l - s1) / cav.v_max;
end
end

function [h, cExtra] = barrier_terms(gap, v, vl, s0, T, cav)
% Barrier value and extra coefficient of u in h_dot for the chosen form.
h = gap - s0 - v * T;
cExtra = 0;
if isfield(cav, 'barrier') && strcmpi(cav.barrier, 'braking') && v > vl
    b = abs(cav.u_min);
    h = h - (v^2 - vl^2) / (2 * b);
    cExtra = v / b;
end
end

function [u, active] = cbf_project(u_nom, h, dv, T, kappa, u_min, u_max, tol, dt)
% Closed-form solution of the reactive safety tracking QP (paper Eq. 18):
%   min 1/2 (u - u_nom)^2
%   s.t. u_min <= u <= u_max
%        h_dot + kappa*h >= 0,   h = gap - s0 - v_cav*T,
%                                h_dot = (v_front - v_cav) - u*T = dv - u*T
% The barrier constraint is linear in u:  u <= (dv + kappa*h)/T.
% Sampled-data form: the command is held for dt, so in addition the
% one-step prediction (leader at constant speed) must satisfy h(t+dt) >= 0:
%   h_next = h + dv*dt - u*(T*dt + dt^2/2) >= 0  ->  u <= (h + dv*dt)/(T*dt + dt^2/2)
% This removes the centimetre overshoot at crawl that the continuous
% condition alone allows. The scalar QP reduces to clipping u_nom to the
% feasible interval. If the bounds lie below u_min the constraint cannot be
% met with the available deceleration; the controller then applies u_min.
if nargin < 9, dt = 0.1; end
u_cbf = (dv + kappa * h) / T;
u_step = (h + dv * dt) / (T * dt + 0.5 * dt^2);
u_bound = min(u_cbf, u_step);
if u_bound < u_nom - tol
    u = u_bound;
    active = true;
else
    u = u_nom;
    active = false;
end
u = min(max(u, u_min), u_max);
end

function T = commit_time_to_cross(d, v, cav)
% Cautious time to cover distance d from speed v: accelerate at
% a_c = u_max*(1 - v/v_max) (full from standstill, none at cruise), capped
% at v_max. Equals d/v at cruise.
v = max(v, 0);
a_c = cav.u_max * max(1 - v / cav.v_max, 0);
if d <= 0
    T = 0; return;
end
if a_c < 1e-6
    T = d / max(v, 0.1); return;
end
t1 = (cav.v_max - v) / a_c;
s1 = v * t1 + 0.5 * a_c * t1^2;
if s1 >= d
    T = (-v + sqrt(v^2 + 2 * a_c * d)) / a_c;
else
    T = t1 + (d - s1) / cav.v_max;
end
end

function tf = crossing_time_ok(laneID, t, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, cav)
% A crossing time is admissible if (i) the lane is green at t AND stays
% green for at least green_margin after t (same margin as the commit rule),
% or (ii) t lies in the clearance interval that follows the CURRENT green of
% this lane and the vehicle, crossing at its estimated speed, clears the
% junction (plus margin) before that clearance interval ends.
if nargin >= 7 && isfield(cav, 'green_margin'), gm = cav.green_margin; else, gm = 1.0; end
tf = lane_green_at_time(laneID, t, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur) && ...
     lane_green_at_time(laneID, t + gm, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur);
if ~tf && nargin >= 7 && isfield(cav, 'v_est')
    [gEnd, yEnd, greenNow, inOwnYellow] = own_window(laneID, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur);
    if (greenNow || inOwnYellow) && t > gEnd && t <= yEnd
        Lj = 21; if isfield(cav, 'junction_length'), Lj = cav.junction_length; end
        tf = (t + gm + Lj / cav.v_est) <= yEnd;
    end
end
end

function [gEnd, yEnd, greenNow, inOwnYellow] = own_window(laneID, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur)
% Current window of this lane, relative to now: gEnd = end of the contiguous
% green (0 if the green has ended and its clearance is running), yEnd = end
% of the clearance interval that follows it. greenNow / inOwnYellow say
% which of the two the lane is in now; both false = lane is red.
gEnd = 0; yEnd = 0; greenNow = false; inOwnYellow = false;
if ~isKey(lanePhaseMap, laneID)
    greenNow = true; gEnd = Inf; yEnd = Inf; return;
end
greenVec = lanePhaseMap(laneID);
first = true;
for i = 1:numel(durSeq)
    if durSeq(i) <= 1e-9, continue; end               % skipped phase
    y = yellow_of_slot(yellowDur, i);
    gp = max(durSeq(i) - y, 0);
    served = greenVec(phaseIdxSeq(i));
    if first
        first = false;
        if ~served, return; end
        if gp > 0, greenNow = true; else, inOwnYellow = true; end
    elseif ~served
        break;
    end
    gEnd = gEnd + gp;
    if y > 0, yEnd = gEnd + y; return; end           % clearance ends the window
end
yEnd = gEnd;
end

function tf = lane_green_at_time(laneID, t, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur)
[phase, isYellow] = phase_at_time(t, phaseIdxSeq, durSeq, yellowDur);

% During yellow tail of any phase slot, no lane is green
if isYellow
    tf = false;
    return;
end

if isKey(lanePhaseMap, laneID)
    laneGreenVec = lanePhaseMap(laneID);
    tf = laneGreenVec(phase);
else
    tf = true;
end
end

function y = yellow_of_slot(yellowDur, i)
% Yellow tail of slot i: per-slot vector or scalar for every slot.
if isscalar(yellowDur)
    y = yellowDur;
elseif i <= numel(yellowDur)
    y = yellowDur(i);
else
    y = max(yellowDur);
end
end

function tEnd = contiguous_green_end(laneID, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur)
% Time (from now) at which the lane's current contiguous green ends.
tEnd = 0;
if isKey(lanePhaseMap, laneID)
    greenVec = lanePhaseMap(laneID);
else
    tEnd = Inf; return;
end
for i = 1:numel(durSeq)
    if durSeq(i) <= 1e-9, continue; end          % skipped phase
    y = yellow_of_slot(yellowDur, i);
    if ~greenVec(phaseIdxSeq(i)), return; end     % lane not served by this slot
    tEnd = tEnd + max(durSeq(i) - y, 0);
    if y > 0, return; end                          % clearance ends the green
end
end

function [phase, isYellow] = phase_at_time(t, phaseIdxSeq, durSeq, yellowDur)
slotDurs = max(durSeq(:)', 0);
totalDur = sum(slotDurs);
isYellow = false;

if t <= totalDur
    acc = 0;
    phase = phaseIdxSeq(end);
    for i = 1:numel(slotDurs)
        if slotDurs(i) <= 1e-9, continue; end     % zero-duration (skipped) phase
        acc = acc + slotDurs(i);
        if t <= acc
            phase = phaseIdxSeq(i);
            % Check if in yellow tail of this slot
            offset_in_slot = t - (acc - slotDurs(i));
            greenPortion = max(slotDurs(i) - yellow_of_slot(yellowDur, i), 0);
            isYellow = (greenPortion <= 0) || (offset_in_slot > greenPortion);
            return;
        end
    end
    return;
end

yellowExt = max(yellowDur);   % clearance assumed after every phase beyond the horizon

nG = max(phaseIdxSeq);
histAvg = phase_history_cache();
if numel(histAvg) < nG
    histAvg = [histAvg, zeros(1, nG - numel(histAvg))];
end

sumDur_plan = zeros(1, nG);
cnt_plan = zeros(1, nG);
for i = 1:numel(phaseIdxSeq)
    p = phaseIdxSeq(i);
    sumDur_plan(p) = sumDur_plan(p) + max(slotDurs(i) - yellow_of_slot(yellowDur, i), 0);   % green portion only
    cnt_plan(p) = cnt_plan(p) + 1;
end

% Extrapolated cycle: each phase gets its historical average GREEN duration
% followed by a clearance interval (yellowExt); phases with zero average
% are skipped.
avgDur = zeros(1, nG);
for p = 1:nG
    if histAvg(p) > 0
        avgDur(p) = histAvg(p);
    elseif cnt_plan(p) > 0
        avgDur(p) = sumDur_plan(p) / cnt_plan(p);
    else
        avgDur(p) = 0;
    end
end
slotExt = avgDur + (avgDur > 0) * yellowExt;

cycleDur = sum(slotExt);
if cycleDur <= 0
    phase = phaseIdxSeq(end); isYellow = true; return;
end
tRel = mod(t - totalDur, cycleDur);
startPhase = mod(phaseIdxSeq(end), nG) + 1;
acc = 0;
phase = phaseIdxSeq(end);

for k = 0:nG-1
    p = mod(startPhase - 1 + k, nG) + 1;
    if slotExt(p) <= 0, continue; end
    acc = acc + slotExt(p);
    if tRel <= acc
        phase = p;
        % Check yellow tail in extrapolated cycle
        offset_in_slot = tRel - (acc - slotExt(p));
        isYellow = offset_in_slot > avgDur(p);
        return;
    end
end
end

function avg = phase_history_cache(newAvg)
persistent cached
if nargin == 1
    cached = newAvg;
end
if isempty(cached)
    cached = [];
end
avg = cached;
end

function t_next_green = find_next_green_start(laneID, t_current, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowDur, cav)
% Find the next time > t_current at which crossing is admissible (green,
% with the crossing-time margin).
if nargin < 8, cav = struct(); end
searchStep = min(0.1, max(0.01, dt));
tMax = t_current + 500;

t = t_current + searchStep;
while t <= tMax
    if crossing_time_ok(laneID, t, lanePhaseMap, phaseIdxSeq, durSeq, yellowDur, cav)
        t_next_green = t;
        return;
    end
    t = t + searchStep;
end

t_next_green = tMax;
end
