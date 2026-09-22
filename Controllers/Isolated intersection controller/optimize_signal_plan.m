function [bestTailDur, bestCost, details] = optimize_signal_plan(vehicles, lanePhaseMap, schedulePhaseIdx, fixedDur4, phaseDurMin, phaseDurMax, baselineTailDur, histAvgDur, idmIn, vehHeadwayIn, yellowDurIn)
% Optimize tail (phases 5..8) for an 8-step rolling horizon.
% Step-1 version:
% - Ideal / signal-free crossing times are estimated lane-by-lane.
% - HDVs use a free-road IDM projection to estimate earliest crossing time.
% - CAVs use the minimum-time optimal-control logic (derived from the
%   attached script) to estimate earliest crossing time.
% - Rear-end interaction in the ideal case is enforced through a temporal
%   headway: t_i >= t_{i-1} + vehHeadway.
% - Signal-aware CAV logic is NOT implemented yet; a placeholder is used.
% - yellowDurIn (optional, default 0): clearance interval shown after every
%   non-zero green that is followed by a different phase (lost time, the
%   epsilon of the paper's Eq. 8). It lengthens the evaluated schedule and
%   is non-green, so the optimizer sees the cost of switching phases.

if nargin < 11 || isempty(yellowDurIn)
    yellowDur = 0;
else
    yellowDur = yellowDurIn;
end

if nargin < 7
    baselineTailDur = [];
end
if nargin >= 8 && ~isempty(histAvgDur)
    phase_history_cache(histAvgDur);   % store for use in phase_at_time
end

if nargin >= 9 && ~isempty(idmIn)
    idm = idmIn;
else
    idm.v0 = 15;
    idm.a = 1.2;
    idm.b = 1.8;
    idm.T = 1.2;
    idm.s0 = 2.0;
    idm.delta = 4;
    idm.L = 5;
end

if ~isfield(idm, 'ab') || isempty(idm.ab)
    idm.ab = sqrt(idm.a * idm.b);
end
if ~isfield(idm, 'inv_v0') || isempty(idm.inv_v0)
    idm.inv_v0 = 1 / idm.v0;
end

if nargin >= 10 && ~isempty(vehHeadwayIn)
    vehHeadway = vehHeadwayIn;
else
    vehHeadway = 2.8;
end

phaseCount = numel(phaseDurMin);
if numel(schedulePhaseIdx) ~= 2 * phaseCount
    error('schedulePhaseIdx must contain exactly 2*phaseCount phase indices.');
end
if numel(fixedDur4) ~= phaseCount
    error('fixedDur4 must contain exactly phaseCount durations.');
end

minTail = zeros(1, phaseCount);
maxTail = zeros(1, phaseCount);
for i = 1:phaseCount
    p = schedulePhaseIdx(phaseCount + i);
    minTail(i) = phaseDurMin(p);
    maxTail(i) = phaseDurMax(p);
end

% Limit search ranges for tail phases based on the number of vehicles
% that can actually benefit from that phase. Lanes served by the same
% phase discharge in parallel, so the bound is the MOST LOADED lane of the
% phase (paper Eq. 8: eta_h = max_l n_{l,h} h_f), not the sum over lanes.
laneCount = containers.Map('KeyType', 'char', 'ValueType', 'double');
for vi = 1:numel(vehicles)
    laneID = vehicles(vi).lane;
    if isKey(laneCount, laneID), laneCount(laneID) = laneCount(laneID) + 1; else, laneCount(laneID) = 1; end
end
vehCountPhase = zeros(1, phaseCount);
laneIDs = keys(laneCount);
for li = 1:numel(laneIDs)
    if isKey(lanePhaseMap, laneIDs{li})
        g = lanePhaseMap(laneIDs{li});
        for ph = 1:numel(g)
            if g(ph)
                vehCountPhase(ph) = max(vehCountPhase(ph), laneCount(laneIDs{li}));
            end
        end
    end
end

% Minimum number of vehicle-slots granted to a phase that is served at all
% (n_min in the paper's Eq. 9c). One-slot greens (h_f = 3.1 s) leave no
% usable window after the crossing-time margin and are dominated by the
% clearance interval; two slots is the smallest useful green.
minServedSlots = 2;

for i = 1:phaseCount
    p = schedulePhaseIdx(phaseCount + i);
    cap = vehCountPhase(p);
    if cap <= 0
        % No vehicles for this phase: hold at minimum.
        maxTail(i) = minTail(i);
    else
        maxTail(i) = min(maxTail(i), cap);
        minTail(i) = max(minTail(i), minServedSlots);
        maxTail(i) = max(maxTail(i), minTail(i));
    end
end

if isempty(baselineTailDur)
    baselineTailDur = (minTail + maxTail) / 2;
end
baselineTailDur = clamp_vec(baselineTailDur, minTail, maxTail);

useParfor = false;   % candidate counts are small (<= coarseCandidateCap); a parpool start (~15 s) would stall a phase

coarseCandidateCap = 2000;

% CAV OC parameters. NOTE: these must equal the cav struct in run_from_matlab.m
% (the optimizer does not receive it). v_max = link speed limit (paper Sec. V);
% the network code had 15 here while the controller used 15 as well.
cav.v_min = 0;
cav.v_max = 13.89;
cav.u_min = -6;
cav.u_max = 5;

% Evaluate a baseline candidate
allDurBaseline = ([fixedDur4(:).', baselineTailDur(:).']) .* vehHeadway;
bestCost = evaluate_total_delay(vehicles, lanePhaseMap, schedulePhaseIdx, allDurBaseline, idm, cav, vehHeadway, 0.2, yellowDur);
if sum(fixedDur4(:))==0
    bestCost = 10^10;
end
bestTailDur = baselineTailDur;

ranges = (maxTail - minTail) + 1;
searchSize = prod(ranges);
candidateCap = coarseCandidateCap;

if searchSize <= candidateCap
    bestCostLocal = bestCost;

    candidates = generate_grid(minTail, maxTail, ones(1, phaseCount));

    nCand = size(candidates, 1);
    costs = inf(nCand, 1);
    hasPar = useParfor && license('test', 'Distrib_Computing_Toolbox');

    if hasPar
        parfor ci = 1:nCand
            cand = candidates(ci, :);
            allDur = ([fixedDur4(:).', cand]) .* vehHeadway;
            costs(ci) = evaluate_total_delay(vehicles, lanePhaseMap, schedulePhaseIdx, allDur, idm, cav, vehHeadway, 0.2, yellowDur);
        end
    else
        for ci = 1:nCand
            cand = candidates(ci, :);
            allDur = ([fixedDur4(:).', cand]) .* vehHeadway;
            costs(ci) = evaluate_total_delay(vehicles, lanePhaseMap, schedulePhaseIdx, allDur, idm, cav, vehHeadway, 0.2, yellowDur);
        end
    end

    [minCost, minIdx] = min(costs);
    if minCost < bestCostLocal
        bestCostLocal = minCost;
        bestTailDur = candidates(minIdx, :);
    end
    bestCost = bestCostLocal;
    details.searchMode = 'exhaustive_integer_grid';
    details.candidateCount = searchSize;
    details.step = 1;

else
    step = max(1, ceil((searchSize / candidateCap)^(1/phaseCount)));
    stepVec = step * ones(1, phaseCount);

    candidates = generate_grid(minTail, maxTail, stepVec);
    approxCount = size(candidates, 1);

    bestCostLocal = bestCost;

    nCand = size(candidates, 1);
    costs = inf(nCand, 1);
    hasPar = useParfor && license('test', 'Distrib_Computing_Toolbox');

    if hasPar
        parfor ci = 1:nCand
            cand = candidates(ci, :);
            allDur = ([fixedDur4(:).', cand]) .* vehHeadway;
            costs(ci) = evaluate_total_delay(vehicles, lanePhaseMap, schedulePhaseIdx, allDur, idm, cav, vehHeadway, 0.2, yellowDur);
        end
    else
        for ci = 1:nCand
            cand = candidates(ci, :);
            allDur = ([fixedDur4(:).', cand]) .* vehHeadway;
            costs(ci) = evaluate_total_delay(vehicles, lanePhaseMap, schedulePhaseIdx, allDur, idm, cav, vehHeadway, 0.2, yellowDur);
        end
    end

    [minCost, minIdx] = min(costs);
    if minCost < bestCostLocal
        bestCostLocal = minCost;
        bestTailDur = candidates(minIdx, :);
    end
    bestCost = bestCostLocal;
    details.searchMode = 'coarse_integer_grid_fallback';
    details.candidateCount = approxCount;
    details.step = step;
end

if searchSize > candidateCap
    improved = true;
    while improved
        improved = false;
        for d = 1:phaseCount
            for dir = [-1, 1]
                trial = bestTailDur;
                trial(d) = clamp_scalar(trial(d) + dir, minTail(d), maxTail(d));
                allDur = ([fixedDur4(:).', trial(:).']) .* vehHeadway;
                c = evaluate_total_delay(vehicles, lanePhaseMap, schedulePhaseIdx, allDur, idm, cav, vehHeadway, 0.2, yellowDur);
                if c < bestCost
                    bestCost = c;
                    bestTailDur = trial;
                    improved = true;
                end
            end
        end
    end
end

allDurBest = ([fixedDur4(:).', bestTailDur(:).']) .* vehHeadway;
bestCost = evaluate_total_delay(vehicles, lanePhaseMap, schedulePhaseIdx, allDurBest, idm, cav, vehHeadway, 0.2, yellowDur);

if ~exist('details', 'var') || isempty(details)
    details = struct();
end
details.minTail = minTail;
details.maxTail = maxTail;
details.baselineTail = baselineTailDur;
if ~isfield(details, 'searchMode')
    details.searchMode = 'exhaustive_integer_grid';
end
if ~isfield(details, 'candidateCount')
    details.candidateCount = NaN;
end
if ~isfield(details, 'step')
    details.step = 1;
end

end

function totalDelay = evaluate_total_delay(vehicles, lanePhaseMap, phaseIdxSeq, durSeq, idm, cav, vehHeadway, dt, yellowDur)
if nargin < 9, yellowDur = 0; end
if isempty(vehicles)
    totalDelay = 0;
    return;
end

% Clearance tails: after every non-zero slot followed by a different phase
% (mirrors the TLS rule in run_from_matlab.m). Slots lengthen by the tail
% and the tail is non-green.
nS = numel(durSeq);
yellowSeq = zeros(1, nS);
for i = 1:nS
    if durSeq(i) <= 0, continue; end
    nextPhase = [];
    for j = i+1:nS
        if durSeq(j) > 0, nextPhase = phaseIdxSeq(j); break; end
    end
    if isempty(nextPhase) || nextPhase ~= phaseIdxSeq(i)
        yellowSeq(i) = yellowDur;
    end
end
durSeq = durSeq(:).' + yellowSeq;

% Lane groups
lanes = {vehicles.lane};
[uniqueLanes, ~, ic] = unique(lanes);
laneGroups = accumarray(ic, (1:numel(vehicles))', [numel(uniqueLanes), 1], @(x) {x'});

% Ideal / no-signal mixed estimator
idealTT = estimate_crossing_times_signal_free(vehicles, laneGroups, idm, cav, vehHeadway, dt);

% Placeholder for the signal-aware estimator
policyTT = estimate_crossing_times_with_signal_placeholder( ...
    vehicles, laneGroups, lanePhaseMap, phaseIdxSeq, durSeq, idm, cav, vehHeadway, dt, yellowSeq);

delay = policyTT - idealTT;
delay(delay < 0) = 0;
totalDelay = sum(delay);
end

function crossingTT = estimate_crossing_times_signal_free(vehicles, laneGroups, idm, cav, vehHeadway, dt)
% Signal-free crossing-time estimator.
% Vehicles are processed lane-by-lane, front to back.
% Earliest arrival:
% - HDV: free-road IDM projection
% - CAV: minimum-time OC arrival
% Rear-end:
% - t_i >= t_{i-1} + vehHeadway

n = numel(vehicles);
crossingTT = inf(n, 1);

for g = 1:numel(laneGroups)
    idx = laneGroups{g};
    if isempty(idx)
        continue;
    end

    dist = zeros(numel(idx), 1);
    for k = 1:numel(idx)
        dist(k) = max(vehicles(idx(k)).distToStop, 0);
    end

    % Front-most first = smallest distance to stop line
    [~, orderLocal] = sort(dist, 'ascend');
    ordered = idx(orderLocal);

    for m = 1:numel(ordered)
        i = ordered(m);

        distToStop = max(vehicles(i).distToStop, 0);
        v0 = max(vehicles(i).speed, 0);

        if isfield(vehicles(i), 'typeID') && strcmpi(vehicles(i).typeID, 'CAV')
            tEarliest = cav_oc_min_time_to_cross(distToStop, v0, cav);
        else
            tEarliest = hdv_idm_free_crossing_time(distToStop, v0, idm, dt);
        end

        if m == 1
            crossingTT(i) = tEarliest;
        else
            j = ordered(m - 1);
            crossingTT(i) = max(tEarliest, crossingTT(j) + vehHeadway);
        end
    end
end
end

function crossingTT = estimate_crossing_times_with_signal_placeholder(vehicles, laneGroups, lanePhaseMap, phaseIdxSeq, durSeq, idm, cav, vehHeadway, dt, yellowSeq)
if nargin < 10, yellowSeq = zeros(size(durSeq)); end
% Signal-aware crossing-time estimator with mixed CAV/HDV logic.
% Vehicles are processed lane-by-lane, front to back.
% Respects both rear-end constraints and signal phases.
%
% Algorithm for each vehicle:
%   1. Compute earliest arrival time (using same physics as signal-free)
%   2. If red blocks arrival, form a queued release process:
%      - first queued vehicle has IDM-based startup delay after green
%      - followers use IDM-based discharge headway
%   3. Apply rear-end constraint with existing predecessor crossing times

n = numel(vehicles);
crossingTT = inf(n, 1);

[startupDelay, queueDischargeHeadway] = idm_queue_release_characteristics(idm, dt, vehHeadway);

for g = 1:numel(laneGroups)
    idx = laneGroups{g};
    if isempty(idx)
        continue;
    end

    % Get distances to stop line for ordering
    dist = zeros(numel(idx), 1);
    for k = 1:numel(idx)
        dist(k) = max(vehicles(idx(k)).distToStop, 0);
    end

    % Front-most first = smallest distance to stop line
    [~, orderLocal] = sort(dist, 'ascend');
    ordered = idx(orderLocal);

    laneID = vehicles(ordered(1)).lane;
    queueCount = 0;
    queueAnchor = NaN;

    for m = 1:numel(ordered)
        i = ordered(m);

        distToStop = max(vehicles(i).distToStop, 0);
        v0 = max(vehicles(i).speed, 0);

        % Compute earliest arrival time using same physics as signal-free
        if isfield(vehicles(i), 'typeID') && strcmpi(vehicles(i).typeID, 'CAV')
            tEarliest = cav_oc_min_time_to_cross(distToStop, v0, cav);
        else
            tEarliest = hdv_idm_free_crossing_time(distToStop, v0, idm, dt);
        end

        % Rear-end feasibility from preceding vehicle crossing
        tReady = tEarliest;
        if m > 1
            j = ordered(m - 1);
            tReady = max(tReady, crossingTT(j) + vehHeadway);
        end

        isGreenAtReady = lane_green_at_time(laneID, tReady, lanePhaseMap, phaseIdxSeq, durSeq, yellowSeq);

        if isGreenAtReady && queueCount == 0
            % No red-formed queue is active for this lane; vehicle can pass at tReady.
            t = tReady;
        else
            % Vehicle is blocked by red or joins an existing queue discharge.
            if queueCount == 0
                tGreen = find_next_green_start(laneID, tReady, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowSeq);
                queueAnchor = tGreen + startupDelay;
                queueCount = 1;
                t = max(tReady, queueAnchor);
            else
                queueCount = queueCount + 1;
                t = max(tReady, queueAnchor + (queueCount - 1) * queueDischargeHeadway);
            end

            % If queue discharge estimate lands on red again, restart queue at next green.
            if ~lane_green_at_time(laneID, t, lanePhaseMap, phaseIdxSeq, durSeq, yellowSeq)
                tGreen = find_next_green_start(laneID, t, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowSeq);
                queueAnchor = tGreen + startupDelay;
                queueCount = 1;
                t = max(t, queueAnchor);
            end
        end

        crossingTT(i) = t;
    end
end
end

function t_next_green = find_next_green_start(laneID, t_current, phaseIdxSeq, durSeq, lanePhaseMap, dt, yellowSeq)
% Find the next time > t_current when the lane is green.
% Analytic within the horizon (walks the slots), stepping search beyond it.
if nargin < 7, yellowSeq = zeros(size(durSeq)); end
if ~isKey(lanePhaseMap, laneID)
    t_next_green = t_current + min(0.1, max(0.01, dt)); return;
end
greenVec = lanePhaseMap(laneID);
acc = 0;
for i = 1:numel(durSeq)
    if durSeq(i) <= 1e-9, continue; end
    slotStart = acc; acc = acc + durSeq(i);
    yi = 0; if i <= numel(yellowSeq), yi = yellowSeq(i); end
    gEndSlot = slotStart + max(durSeq(i) - yi, 0);
    if greenVec(phaseIdxSeq(i)) && gEndSlot > t_current
        t_next_green = max(slotStart, t_current + 1e-3);
        return;
    end
end
% beyond the horizon: stepping search on the extrapolated cycle
searchStep = min(0.1, max(0.01, dt));
tMax = t_current + 500;
t = max(acc, t_current) + searchStep;
while t <= tMax
    if lane_green_at_time(laneID, t, lanePhaseMap, phaseIdxSeq, durSeq, yellowSeq)
        t_next_green = t;
        return;
    end
    t = t + searchStep;
end
t_next_green = tMax;
end

function tCross = hdv_idm_free_crossing_time(distToStop, v0, idm, ~)
% Analytical free-road IDM crossing time (no leader, no signal).
% Two-phase approximation: constant acceleration at the initial IDM
% free-road rate until reaching desired speed, then cruise.

if distToStop <= 0
    tCross = 0;
    return;
end

v = max(v0, 0);

% Free-road IDM acceleration at current speed
a0 = idm.a * (1 - (v * idm.inv_v0)^idm.delta);

if a0 < 1e-6
    % At or above desired speed — cruise at current speed
    tCross = distToStop / max(v, 0.01);
    return;
end

% Phase 1: constant acceleration a0 until reaching desired speed
t_acc = (idm.v0 - v) / a0;
d_acc = v * t_acc + 0.5 * a0 * t_acc^2;

if d_acc >= distToStop
    % Target reached during acceleration phase — quadratic formula
    disc = v^2 + 2 * a0 * distToStop;
    tCross = (-v + sqrt(max(disc, 0))) / a0;
else
    % Phase 2: cruise at v_desired for remaining distance
    tCross = t_acc + (distToStop - d_acc) / idm.v0;
end
end

function a = idm_free_accel(v, idm)
freeTerm = (v * idm.inv_v0) .^ idm.delta;
a = idm.a * (1 - freeTerm);
end

function T_min = cav_oc_min_time_to_cross(l, v0, cav)
% Minimum-time arrival for the CAV based on the attached optimal-control logic.
% This is the earliest feasible time to cover distance l from speed v0,
% under acceleration bound cav.u_max and speed bound cav.v_max.

if l <= 0
    T_min = 0;
    return;
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
% Estimate queue release parameters from IDM using a two-vehicle launch.
% - startupDelay: first queued vehicle time to cross after green
% - queueHeadway: time gap between first and second queued crossings

leadLength = max(idm.L, 0.1);
xLead = -leadLength;
vLead = 0;

xFollow = xLead - (idm.L + idm.s0);
vFollow = 0;

t = 0;
tMax = 120;
nSteps = max(2, ceil(tMax / dt));

tLeadCross = NaN;
tFollowCross = NaN;

for k = 1:nSteps
    aLead = idm_free_accel(vLead, idm);
    vLeadNext = max(vLead + aLead * dt, 0);
    xLeadNext = xLead + vLead * dt + 0.5 * aLead * dt^2;

    aFollow = idm_accel(vFollow, vLead, xFollow, xLead, idm);
    vFollowNext = max(vFollow + aFollow * dt, 0);
    xFollowNext = xFollow + vFollow * dt + 0.5 * aFollow * dt^2;

    if isnan(tLeadCross) && xLeadNext >= 0
        if xLeadNext > xLead
            frac = (0 - xLead) / (xLeadNext - xLead);
            frac = min(max(frac, 0), 1);
            tLeadCross = t + frac * dt;
        else
            tLeadCross = t + dt;
        end
    end

    if isnan(tFollowCross) && xFollowNext >= 0
        if xFollowNext > xFollow
            frac = (0 - xFollow) / (xFollowNext - xFollow);
            frac = min(max(frac, 0), 1);
            tFollowCross = t + frac * dt;
        else
            tFollowCross = t + dt;
        end
    end

    xLead = xLeadNext;
    vLead = vLeadNext;
    xFollow = xFollowNext;
    vFollow = vFollowNext;
    t = t + dt;

    if ~isnan(tLeadCross) && ~isnan(tFollowCross)
        break;
    end
end

if isnan(tLeadCross)
    tLeadCross = max(dt, 1.0);
end
if isnan(tFollowCross)
    tFollowCross = tLeadCross + max(idm.T, dt);
end

startupDelay = max(tLeadCross, dt);
queueHeadway = max([tFollowCross - tLeadCross, minHeadway, dt]);
end

function tf = lane_green_at_time(laneID, t, lanePhaseMap, phaseIdxSeq, durSeq, yellowSeq)
if nargin < 6, yellowSeq = zeros(size(durSeq)); end
[phase, isYellow] = phase_at_time(t, phaseIdxSeq, durSeq, yellowSeq);
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

function [phase, isYellow] = phase_at_time(t, phaseIdxSeq, durSeq, yellowSeq)
clampedDurs = max(durSeq(:)', 0);
totalDur = sum(clampedDurs);
isYellow = false;
yellowSeq = yellowSeq(:)';

if t <= totalDur
    acc = 0;
    phase = phaseIdxSeq(end);
    for i = 1:numel(clampedDurs)
        if clampedDurs(i) <= 1e-9, continue; end   % zero-duration (skipped) phase
        acc = acc + clampedDurs(i);
        if t <= acc
            phase = phaseIdxSeq(i);
            offset_in_slot = t - (acc - clampedDurs(i));
            yi = 0; if i <= numel(yellowSeq), yi = yellowSeq(i); end
            isYellow = offset_in_slot > max(clampedDurs(i) - yi, 0);
            return;
        end
    end
    return;
end
yellowExt = 0; if ~isempty(yellowSeq), yellowExt = max(yellowSeq); end

nG = max(phaseIdxSeq);
histAvg = phase_history_cache();
if numel(histAvg) < nG
    histAvg = [histAvg, zeros(1, nG - numel(histAvg))];
end

sumDur_plan = zeros(1, nG);
cnt_plan = zeros(1, nG);
for i = 1:numel(phaseIdxSeq)
    p = phaseIdxSeq(i);
    yi = 0; if i <= numel(yellowSeq), yi = yellowSeq(i); end
    sumDur_plan(p) = sumDur_plan(p) + max(clampedDurs(i) - yi, 0);   % green portion only
    cnt_plan(p) = cnt_plan(p) + 1;
end

% Extrapolated cycle: historical average GREEN per phase followed by a
% clearance interval; phases with zero average are skipped.
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

function out = clamp_vec(x, lo, hi)
out = min(max(x, lo), hi);
end

function y = clamp_scalar(x, lo, hi)
y = min(max(x, lo), hi);
end

function candidates = generate_grid(lo, hi, stepVec)
% Generate all grid points in the hyper-rectangle [lo, hi] with given steps.
% Returns an N x D matrix where D = numel(lo).
D = numel(lo);
ranges = cell(1, D);
for d = 1:D
    ranges{d} = lo(d):stepVec(d):hi(d);
    if isempty(ranges{d})
        ranges{d} = lo(d);
    end
end
grids = cell(1, D);
[grids{:}] = ndgrid(ranges{:});
nPts = numel(grids{1});
candidates = zeros(nPts, D);
for d = 1:D
    candidates(:, d) = grids{d}(:);
end
end
