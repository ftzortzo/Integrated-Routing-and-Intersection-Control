preliminaries

% =========================================================================
% ISOLATED-INTERSECTION HARNESS FOR THE PAPER'S LOWER-LAYER CONTROLLER
% =========================================================================
% This script is the Sioux Falls run script
%   FINAL\Final_play - Copy (5)\run_from_matlab.m
% reduced to ONE signalized intersection ('C' in quickstart.net.xml), with
% the high-level routing removed (vehicles come from flows in
% quickstart.rou.xml) and per-vehicle trajectory logging added.
%
% The two controller files it calls are VERBATIM copies of the ones that
% produced the paper's results:
%   cav_control_policy.m    (lower layer: CAV trajectories, Sec. III-D/E)
%   optimize_signal_plan.m  (lower layer: signal phase optimization, Sec. III-C)
%
% Per-step structure (identical to the network script):
%   GATHER  -> batch-query all vehicles via TraCI subscriptions
%   COMPUTE -> build phase horizon, run cav_control_policy, advance signal,
%              run optimize_signal_plan at each phase start
%   APPLY   -> execute the deferred TraCI commands
% -------------------------------------------------------------------------

% =========================================================================
% RUN FLAGS
% =========================================================================
OPTIMIZE_SIGNALS = true;      % true: adaptive signal optimization (paper)
                              % false: fixed-time baseline
fixedGreenDuration  = 25;     % [s] green per phase (baseline only)
fixedYellowDuration = 2;      % [s] yellow per phase (baseline only)
if exist('SIM_DURATION_OVERRIDE', 'var')
    simDuration_s = SIM_DURATION_OVERRIDE;   % set in the workspace to override
else
    simDuration_s = 300;      % [s] total simulation time (paper runs: 2100)
end

% How the CAV commands are applied to SUMO (see PHASE 3 below):
%   'fixed'         -> setSpeed only; SUMO's speed/right-of-way/red-light/
%                      speed-limit checks disabled for CAVs inside the control
%                      zone (speedMode 96) and lane changes frozen (512), so
%                      the CAV executes exactly the controller's command and
%                      SUMO integrates the position from it.  DEFAULT.
%   'legacy_moveTo' -> setSpeed + moveTo, as in the network script that
%                      produced the paper's results. Kept only for comparison:
%                      it moves each CAV twice per step (see check_kinematics).
APPLY_MODE = 'fixed';

% =========================================================================
% CONNECT TO SUMO
% =========================================================================
[traciV, sumoV] = traci.init(port, 10, '127.0.0.1', 'default');
disp(traciV); disp(sumoV);

% =========================================================================
% INTERSECTION CONFIGURATION (single intersection)
% =========================================================================
% controlZoneRadius: the paper states 200 m. The current network script
% has 150 m (it was edited after the results were produced), the Mar-31
% snapshot has 200 m.
% phaseDurMax is derived below from the zone's storage capacity
% (n_max = floor((R - l_c)/(L + s0)) vehicle-slots, Eq. 9c); the value here
% is a placeholder.
intxDefs = { ...
    struct('id', 'C', ...
           'phases', {{'GGrrrrGGrrrr', 'rrGrrrrrGrrr', 'rrrGGrrrrGGr', 'rrrrrGrrrrrG'}}, ...
           'phaseDurMin', [0, 0, 0, 0], ...
           'phaseDurMax', [25, 25, 25, 25], ...
           'controlZoneRadius', 200) ...
};

% =========================================================================
% SHARED PARAMETERS (values of FINAL\Final_play - Copy (5))
% =========================================================================
% Conversion parameter between vehicle counts (eta) and seconds.
vehicleHeadway = 3.1;

% Duration [s] of the clearance interval (yellow) after every non-zero green
% phase: h_c in the paper's Eq. (2). Must be long enough for a vehicle that
% enters the junction at the end of green to clear it (the 4-leg junction is
% ~20 m across: >= 1.5 s at 13.89 m/s, 2-3 s when launching from the queue).
% The network runs used 0.5 s and relied on SUMO's right-of-way logic to
% hold vehicles; with SUMO's checks disabled for CAVs, h_c has to do this.
if OPTIMIZE_SIGNALS
    yellowDuration = 3.0;
else
    yellowDuration = fixedYellowDuration;
end

% Headway parameter for CAV crossing time calculations.
cavHeadway = 0.5;

% IDM parameters for HDVs (prediction model used by the controllers)
idm.v0 = 13.89;      % desired speed = link speed limit (paper Sec. V); code had 15
idm.a = 3;
idm.b = 1.8;
idm.T = 2.5;
idm.s0 = 2.0;
idm.delta = 4;
idm.L = 5;
idm.ab = sqrt(idm.a * idm.b);
idm.inv_v0 = 1 / idm.v0;

% CAV optimal control parameters
cav.v_min = 0;
cav.v_max = 13.89;   % max speed = link speed limit (paper Sec. V); code had 15
cav.u_min = -6;     % paper Sec. V: u_min = -6 m/s^2 (the network code had -3, which is
                    % infeasible behind an HDV braking at 6 m/s^2 with a 0.5 s headway)
if exist('UMIN_OVERRIDE', 'var'), cav.u_min = UMIN_OVERRIDE; end
cav.u_max = 5;      % paper Sec. V: u_max = 5 m/s^2 (network code had 3.459). Also the CAV vType accel.
% Rear-end safety constraint (paper Eqs. 13, 18): h = gap - s0 - v*T_safe,
% enforced through h_dot + kappa*h >= 0 inside cav_control_policy.
cav.T_safe = 0.5;   % safe time headway T [s]. Feasibility of the first-order barrier behind
                    % an HDV braking at b_HDV requires s0 + v*T >= v^2/(2|u_min|) - v^2/(2 b_HDV);
                    % with |u_min| = b_HDV = 6 this holds for any T >= 0. (With |u_min| = 3 it
                    % would require T >= 1.2 s, tested: safe but ~7 s slower per CAV trip.)
cav.kappa  = 0.3;   % class-K gain kappa [1/s] (tunable)
% Comfort headway [s] used to shape the nominal control after the stop line
% (downstream policy): the CAV eases off toward s0 + v*T_comfort well before
% the safety constraint (T_safe) becomes active.
cav.T_comfort = 1.5;
if exist('KAPPA_OVERRIDE', 'var'), cav.kappa = KAPPA_OVERRIDE; end
% Rear-end barrier form: 'headway' (paper Eq. 13) or 'braking' (Eq. 13 plus the
% CAV's braking distance to the leader's speed at |u_min|: input-constrained form).
cav.barrier = 'headway';   % paper Eq. 13. 'braking' tested (Sept 21): safe up to kappa = 1.0,
                            % no efficiency change (the residual CAV-HDV gap is yellow-entry asymmetry).
if exist('BARRIER_OVERRIDE', 'var'), cav.barrier = BARRIER_OVERRIDE; end
% Leader crossing-time estimate in the CAV target selection: 'model' = planned
% target of a CAV leader / closed-form IDM prediction of an HDV leader (default);
% 'mintime' = leader's minimum-time crossing (previous behaviour).
cav.leaderPred = 'model';
if exist('LEADERPRED_OVERRIDE', 'var'), cav.leaderPred = LEADERPRED_OVERRIDE; end
if exist('TSAFE_OVERRIDE', 'var'), cav.T_safe = TSAFE_OVERRIDE; end
% Dilemma-zone margin [s]: a CAV commits to crossing in the current green
% only if its planned crossing lies at least this long before the green
% ends (and, with a leader, only if it makes it at its current speed).
cav.green_margin = 1.0;
if exist('GREEN_MARGIN_OVERRIDE', 'var'), cav.green_margin = GREEN_MARGIN_OVERRIDE; end
% Margin used when the crossing is deterministic (no leader or a CAV leader,
% paper Remark 1); the min-time plan is tracked exactly, so a small value
% covering the discretisation suffices.
cav.green_margin_det = 0.3;
% Standoff [m] before the stop line at which a CAV comes to rest on red, so
% that discretisation never carries its front bumper onto the internal lane.
cav.line_standoff = 1.0;
% Junction length [m] used to decide whether an entry during the clearance
% interval still clears the junction before the conflicting green starts
% (quickstart.net.xml: straight 20.8 m, left turn 19.4 m).
cav.junction_length = 21;

dt = 0.1;   % SUMO timestep in seconds (must match preliminaries.m and quickstart.sumocfg)

presortEveryN = 100;   % presort cadence in steps (network script: every 100 steps)

% =========================================================================
% INITIALIZE PER-INTERSECTION RUNTIME STATE
% =========================================================================
numIntx = numel(intxDefs);
intx = cell(1, numIntx);

for ix = 1:numIntx
    s = intxDefs{ix};
    nP = numel(s.phases);
    s.nPhases = nP;
    % Maximum green per phase = storage capacity of the zone on one lane
    % (vehicles that fit queued between the zone entrance and the stop line,
    % l_c ~ 10 m from the junction centre) times the discharge headway.
    stopLineOffset = 10;
    nMaxZone = floor((s.controlZoneRadius - stopLineOffset) / (idm.L + idm.s0));   % zone storage capacity (27 at R = 200)
    % With snapshot-based (no arrival forecast) allocation, a green as long
    % as the full zone storage starves the other approaches; n_max is
    % therefore capped at a moderate value that acts as a maximum green.
    nMaxZone = min(nMaxZone, 14);
    if exist('N_MAX_OVERRIDE', 'var'), nMaxZone = N_MAX_OVERRIDE; end
    s.nMaxZone = nMaxZone;
    s.phaseDurMax = nMaxZone * vehicleHeadway * ones(1, nP);
    fprintf('[%s] n_max = %d vehicle-slots (phaseDurMax = %.1f s) from R = %d m\n', s.id, nMaxZone, s.phaseDurMax(1), s.controlZoneRadius);
    if OPTIMIZE_SIGNALS
        s.phaseDurNom = 0.1 * (s.phaseDurMin + s.phaseDurMax);
    else
        s.phaseDurNom = fixedGreenDuration * ones(1, nP);
    end

    % Identify signal positions that are green in ALL phases (always-green).
    phaseLen = length(s.phases{1});
    s.alwaysGreenMask = true(1, phaseLen);
    for pi = 1:nP
        for ci = 1:phaseLen
            ch = s.phases{pi}(ci);
            if ch ~= 'G' && ch ~= 'g'
                s.alwaysGreenMask(ci) = false;
            end
        end
    end

    % Generate yellow-phase strings: only non-always-green 'G'/'g' become 'y'.
    s.yellowPhases = s.phases;
    for ypi = 1:nP
        yStr = s.phases{ypi};
        for ci = 1:phaseLen
            if ~s.alwaysGreenMask(ci) && (yStr(ci) == 'G' || yStr(ci) == 'g')
                yStr(ci) = 'y';
            end
        end
        s.yellowPhases{ypi} = yStr;
    end

    % Runtime schedule state
    s.currentPhaseIdx   = 1;
    s.currentPhaseStart = 0;
    s.currentPhaseDur   = s.phaseDurNom(1);
    s.futureDur         = zeros(1, 2*nP - 1);
    for j = 1:(2*nP - 1)
        p = mod(j, nP) + 1;
        s.futureDur(j) = s.phaseDurNom(p);
    end

    s.phaseHistSum = zeros(1, nP);
    s.phaseHistCnt = zeros(1, nP);
    s.yellowEndTime = 0;
    s.yellowFromPhase = [];

    % Set initial TLS state
    traci.trafficlights.setRedYellowGreenState(s.id, s.phases{s.currentPhaseIdx});
    fprintf('[%s] Initial phase %d for %.2fs\n', s.id, s.currentPhaseIdx, s.currentPhaseDur);

    intx{ix} = s;
end

% =========================================================================
% INITIAL OPTIMIZATION FOR EACH INTERSECTION
% =========================================================================
if OPTIMIZE_SIGNALS
for ix = 1:numIntx
    s = intx{ix};
    nP = s.nPhases;

    [vehiclesInZone, lanePhaseMap] = collect_control_zone_state(s.id, s.controlZoneRadius, s.phases);
    vehiclesForOpt = filter_always_green_vehicles(vehiclesInZone, lanePhaseMap);
    if ~isempty(vehiclesForOpt)
        fixedDur = [s.currentPhaseDur, s.futureDur(1:nP-1)];
        fixedCounts = max(0, ceil(fixedDur ./ vehicleHeadway));
        schedulePhaseIdx = zeros(1, 2*nP);
        for i = 1:2*nP
            schedulePhaseIdx(i) = mod(s.currentPhaseIdx - 1 + (i-1), nP) + 1;
        end
        baselineTailCounts = max(0, ceil(s.futureDur(nP:2*nP-1) ./ vehicleHeadway));
        phaseVehMin = max(0, floor(s.phaseDurMin ./ vehicleHeadway));
        phaseVehMax = max(1, ceil(s.phaseDurMax ./ vehicleHeadway));
        fprintf('[%s] Initial optimization at t=0 with %d vehicles (%d signal-dependent)\n', s.id, numel(vehiclesInZone), numel(vehiclesForOpt));
        tic
        [bestTailCounts, bestCost, ~] = optimize_signal_plan( ...
            vehiclesForOpt, lanePhaseMap, schedulePhaseIdx, fixedCounts, ...
            phaseVehMin, phaseVehMax, baselineTailCounts, ...
            s.phaseHistSum ./ max(s.phaseHistCnt, 1), idm, vehicleHeadway, yellowDuration);
        toc
        s.futureDur(nP:2*nP-1) = bestTailCounts .* vehicleHeadway;

        vehCountByPhase0 = count_vehicles_per_phase(vehiclesForOpt, lanePhaseMap, nP);
        for fi0 = 1:nP-1
            futurePhase0 = mod(s.currentPhaseIdx - 1 + fi0, nP) + 1;
            if vehCountByPhase0(futurePhase0) == 0
                s.futureDur(fi0) = 0;
            end
        end

        fprintf('[%s]   initial best cost=%.3f\n', s.id, bestCost);
    end

    intx{ix} = s;
end
end % if OPTIMIZE_SIGNALS

% =========================================================================
% DATA COLLECTION INITIALIZATION
% =========================================================================
% Two paths are tracked for the time-space diagrams (plot_trajectories.m):
%   Path 1: N->E  incoming edge NC, lanes NC_0/NC_1, routes 'ne'
%   Path 2: E->W  incoming edge EC, lanes EC_0/EC_1, routes 'ew'
pathDefs = struct();
pathDefs(1).name = 'N \rightarrow E';
pathDefs(1).incomingEdge = 'NC';
pathDefs(1).incomingLanes = {'NC_0', 'NC_1'};
pathDefs(1).outgoingEdge = 'CE';
pathDefs(1).routePrefix = 'ne';
pathDefs(1).greenPhases = [2];
pathDefs(1).tlsLinkIndices = [3];      % 1-based position in the TLS string of the N LEFT-turn head (link 2) only
pathDefs(2).name = 'E \rightarrow W';
pathDefs(2).incomingEdge = 'EC';
pathDefs(2).incomingLanes = {'EC_0', 'EC_1'};
pathDefs(2).outgoingEdge = 'CW';
pathDefs(2).routePrefix = 'ew';
pathDefs(2).greenPhases = [3];
pathDefs(2).tlsLinkIndices = [5];      % E STRAIGHT head (link 4) only

recordEveryN = 5;   % record every 5 steps = every 0.5 s at dt = 0.1

% Trajectory log: time, vehicle id, path index, signed distance to stop
% line (negative = approaching), reported speed, type.
trajLog = struct('t', {}, 'vid', {}, 'pathIdx', {}, 'dist', {}, 'speed', {}, 'typeID', {});
trajLogIdx = 0;

% Actual TLS state log (every recording interval)
tlsLog = [];
tlsStates = {};

% Optimization event log
optLog = struct('t', {}, 'phaseIdx', {}, 'nVehicles', {}, 'cost', {}, ...
    'fixedDur', {}, 'tailCounts', {}, 'futureDur', {}, ...
    'vehPerPhase', {}, 'stoppedPerPhase', {}, 'vehPerPhaseAll', {}, 'stoppedPerPhaseAll', {}, ...
    'fixedCounts', {}, 'futureDurAfter', {}, 'currentPhaseDur', {}, 'searchMode', {}, 'candidateCount', {}, 'step', {}, 'wallTime', {});
optLogIdx = 0;

% Per-second queue log per incoming lane (all vehicles on the lane, not only
% inside the zone): total and stopped counts.
qLanes = {'NC_0','NC_1','EC_0','EC_1','SC_0','SC_1','WC_0','WC_1'};
qLogCap = 5000; qLog.t = zeros(qLogCap,1); qLog.total = zeros(qLogCap, numel(qLanes)); qLog.stopped = zeros(qLogCap, numel(qLanes));
qLog.inZone = zeros(qLogCap, numel(qLanes)); qLog.phase = zeros(qLogCap,1); qLog.tls = cell(qLogCap,1); qLogN = 0;

% Trip records (depart/arrive per vehicle, from SUMO departed/arrived lists)
tripDepart = containers.Map('KeyType','char','ValueType','double');
tripType   = containers.Map('KeyType','char','ValueType','char');
tripRecords = struct('id',{},'depart',{},'arrive',{},'type',{});
nTrips = 0;

% Incoming-lane lengths for odometer-based signed distance
pathInEdgeLen = zeros(1, numel(pathDefs));
for pi = 1:numel(pathDefs)
    try
        pathInEdgeLen(pi) = traci.lane.getLength(pathDefs(pi).incomingLanes{1});
    catch
        pathInEdgeLen(pi) = 489.6;  % fallback (quickstart.net.xml)
    end
end

% Bookkeeping for APPLY_MODE = 'fixed' (CAVs currently under external control)
cavUnderControl = containers.Map('KeyType','char','ValueType','logical');
nDownCmds = 0;     % downstream-policy commands issued
nDownLeader = 0;   % ... of which with a leader found
polTimeSum = 0; polTimeMax = 0; polCalls = 0;   % CAV controller wall time per step

% Per-step CAV log (every step, every CAV): used by analyze_braking.m to
% attribute every large speed drop to its cause (controller command,
% SUMO-driven before the zone, hand-back on the outgoing lane).
stepLogCap = 400000;
stepLog.t          = zeros(stepLogCap, 1);
stepLog.vid        = cell(stepLogCap, 1);
stepLog.lane       = cell(stepLogCap, 1);
stepLog.speed      = zeros(stepLogCap, 1);     % speed reported by SUMO this step
stepLog.underCtrl  = false(stepLogCap, 1);     % in cavUnderControl at this step
stepLog.cmd_u      = nan(stepLogCap, 1);       % controller u (NaN if no command)
stepLog.cmd_vnext  = nan(stepLogCap, 1);       % commanded v_next (NaN if no command)
stepLog.distToStop = nan(stepLogCap, 1);
stepLog.lanePos    = nan(stepLogCap, 1);
% controller diagnostics (NaN/empty when no command this step)
stepLog.t_target   = nan(stepLogCap, 1);
stepLog.u_nom      = nan(stepLogCap, 1);
stepLog.cbf_active = false(stepLogCap, 1);
stepLog.h          = nan(stepLogCap, 1);
stepLog.gap        = nan(stepLogCap, 1);
stepLog.leader     = cell(stepLogCap, 1);
stepLog.mustStop   = false(stepLogCap, 1);
stepLog.stopActive = false(stepLogCap, 1);
stepLog.greenNow   = false(stepLogCap, 1);
stepLog.greenEnd   = nan(stepLogCap, 1);
stepLog.profile    = cell(stepLogCap, 1);
stepLog.standingStart = false(stepLogCap, 1);
stepLog.lineClamp  = false(stepLogCap, 1);
stepLog.replanned  = false(stepLogCap, 1);
stepLog.tls        = cell(stepLogCap, 1);      % TLS state string at this step
stepLogN = 0;
% downstream-policy log (after the stop line)
dsLog.t = zeros(stepLogCap,1); dsLog.vid = cell(stepLogCap,1); dsLog.lane = cell(stepLogCap,1);
dsLog.v = zeros(stepLogCap,1); dsLog.gap = nan(stepLogCap,1); dsLog.u = nan(stepLogCap,1); dsLog.leader = cell(stepLogCap,1);
dsLogN = 0;
% mid-junction stop log
jStop = repmat(struct('t', 0, 'vid', '', 'type', '', 'lane', '', 'lanePos', 0, 'tls', '', 'others', ''), 1, 2000);
jStopN = 0;

% =========================================================================
% MAIN SIMULATION LOOP
% =========================================================================
nSteps = round(simDuration_s / dt);
for k = 1:nSteps

    try
        traci.simulationStep();
    catch ME
        warning(ME.identifier, '%s', ME.message);
        break;
    end

    try
        tnow = traci.simulation.getTime();
    catch
        tnow = k * dt;
    end

    % --- Yellow -> Green transitions for all intersections ---
    for ix = 1:numIntx
        s = intx{ix};
        if s.yellowEndTime > 0 && tnow >= s.yellowEndTime
            try
                traci.trafficlights.setRedYellowGreenState(s.id, s.phases{s.currentPhaseIdx});
            catch ME
                warning(ME.identifier, '%s', ME.message);
            end
            s.yellowEndTime = 0;
            intx{ix} = s;
        end
    end

    if mod(k, presortEveryN) == 0
        presort
    end

    % =================================================================
    % DATA COLLECTION: departures / arrivals
    % =================================================================
    try
        departedIDs = traci.simulation.getDepartedIDList();
    catch
        departedIDs = {};
    end
    for di = 1:numel(departedIDs)
        vid_dep = departedIDs{di};
        tripDepart(vid_dep) = tnow;
        try
            tripType(vid_dep) = traci.vehicle.getTypeID(vid_dep);
        catch
            tripType(vid_dep) = 'unknown';
        end
    end
    try
        arrivedIDs = traci.simulation.getArrivedIDList();
    catch
        arrivedIDs = {};
    end
    for ai = 1:numel(arrivedIDs)
        vid_arr = arrivedIDs{ai};
        if isKey(tripDepart, vid_arr)
            nTrips = nTrips + 1;
            tripRecords(nTrips).id     = vid_arr;
            tripRecords(nTrips).depart = tripDepart(vid_arr);
            tripRecords(nTrips).arrive = tnow;
            tripRecords(nTrips).type   = tripType(vid_arr);
            remove(tripDepart, vid_arr);
            remove(tripType,   vid_arr);
        end
        if isKey(cavUnderControl, vid_arr)
            remove(cavUnderControl, vid_arr);
        end
    end

    % =================================================================
    % PHASE 1: GATHER - batch-query all vehicle state from SUMO once
    % =================================================================
    allVehData = batch_query_all_vehicles();
    laneVehIndex = build_lane_vehicle_index(allVehData);

    intxVehicles = cell(1, numIntx);
    intxLanePhaseMaps = cell(1, numIntx);
    intxJunctionPos = cell(1, numIntx);
    for ix = 1:numIntx
        s = intx{ix};
        [intxVehicles{ix}, intxLanePhaseMaps{ix}] = collect_control_zone_state(s.id, s.controlZoneRadius, s.phases, allVehData, laneVehIndex);
        try intxJunctionPos{ix} = traci.junction.getPosition(s.id); catch; intxJunctionPos{ix} = []; end
    end

    % =================================================================
    % DATA COLLECTION: TLS state + trajectories (every recordEveryN steps)
    % =================================================================
    if mod(k, 10) == 0 && qLogN < qLogCap
        qLogN = qLogN + 1;
        qLog.t(qLogN) = tnow; qLog.phase(qLogN) = intx{1}.currentPhaseIdx;
        try qLog.tls{qLogN} = traci.trafficlights.getRedYellowGreenState(intx{1}.id); catch, qLog.tls{qLogN} = ''; end
        posQ = intxJunctionPos{1};
        for li = 1:numel(qLanes)
            if isKey(laneVehIndex, qLanes{li})
                ids = laneVehIndex(qLanes{li});
                qLog.total(qLogN, li) = numel(ids);
                sp = [allVehData(ids).speed];
                qLog.stopped(qLogN, li) = nnz(sp < 0.5);
                nz = 0;
                for ii = ids
                    p = allVehData(ii).pos;
                    if ~isempty(p) && ~isempty(posQ) && hypot(p(1)-posQ(1), p(2)-posQ(2)) <= intx{1}.controlZoneRadius, nz = nz + 1; end
                end
                qLog.inZone(qLogN, li) = nz;
            end
        end
    end
    if mod(k, recordEveryN) == 0
        try
            tlsStateNow = traci.trafficlights.getRedYellowGreenState(intx{1}.id);
        catch
            tlsStateNow = '';
        end
        if ~isempty(tlsStateNow)
            tlsLog(end+1) = tnow; %#ok<AGROW>
            tlsStates{end+1} = tlsStateNow; %#ok<AGROW>
        end

        for vi_rec = 1:numel(allVehData)
            vd = allVehData(vi_rec);
            if isempty(vd.lane), continue; end
            try
                routeID_rec = traci.vehicle.getRouteID(vd.id);
                odom_rec = traci.vehicle.getDistance(vd.id);
            catch
                continue;
            end
            matchedPath = 0;
            for pi = 1:numel(pathDefs)
                if contains(lower(routeID_rec), pathDefs(pi).routePrefix)
                    matchedPath = pi;
                    break;
                end
            end
            if matchedPath == 0, continue; end

            % Signed distance to stop line via odometer: negative = approaching
            signedDist = odom_rec - pathInEdgeLen(matchedPath);
            if abs(signedDist) > intx{1}.controlZoneRadius + 50, continue; end

            trajLogIdx = trajLogIdx + 1;
            trajLog(trajLogIdx) = struct('t', tnow, 'vid', vd.id, ...
                'pathIdx', matchedPath, 'dist', signedDist, 'speed', vd.speed, ...
                'typeID', vd.typeID);
        end
    end

    % =================================================================
    % PHASE 2: COMPUTE - per-intersection processing (TraCI-free)
    % =================================================================
    allCavCmds = cell(1, numIntx);
    allResetCmds = cell(1, numIntx);
    allTlsCmds = cell(1, numIntx);
    for ix = 1:numIntx
        s = intx{ix};
        vehiclesInZone = intxVehicles{ix};
        lanePhaseMap = intxLanePhaseMaps{ix};
        posZ = intxJunctionPos{ix};
        nP = s.nPhases;
        tlsCmds = {};

        % Build phase sequence for CAV control (2*nP phases ahead)
        horizonLen = 2 * nP;
        phaseIdxSeq = zeros(1, horizonLen);
        for i = 1:horizonLen
            phaseIdxSeq(i) = mod(s.currentPhaseIdx - 1 + (i - 1), nP) + 1;
        end

        % Build the phase horizon handed to the CAVs: slot durations INCLUDING
        % their clearance (yellow) tails, plus a per-slot vector of those tails.
        % A slot gets a yellow tail only where the real TLS will show one:
        % the slot has non-zero duration AND the next non-zero slot is a
        % different phase (zero-duration phases are skipped without any
        % yellow, and a phase re-granted after skipped phases is one
        % continuous green). When yellow is active now, a yellow-only slot
        % (tagged with the old phase) is prepended.
        if s.yellowEndTime > 0 && tnow < s.yellowEndTime
            remainingYellow = s.yellowEndTime - tnow;
            % The clearance being shown belongs to the phase that was green
            % before the switch (NOT necessarily currentPhaseIdx-1: skipped
            % zero-duration phases sit in between).
            if isfield(s, 'yellowFromPhase') && ~isempty(s.yellowFromPhase)
                prevPhaseIdx = s.yellowFromPhase;
            else
                prevPhaseIdx = mod(s.currentPhaseIdx - 2, nP) + 1;
            end
            [durTail, yelTail] = build_signal_horizon(phaseIdxSeq, [s.currentPhaseDur, s.futureDur], yellowDuration);
            phaseIdxSeq = [prevPhaseIdx, phaseIdxSeq];
            durSeq   = [remainingYellow, durTail];
            yellowSeq = [remainingYellow, yelTail];
        else
            remainingCurrentPhaseDur = max(s.currentPhaseStart + s.currentPhaseDur - tnow, 0);
            [durSeq, yellowSeq] = build_signal_horizon(phaseIdxSeq, [remainingCurrentPhaseDur, s.futureDur], yellowDuration);
        end

        if isempty(vehiclesInZone)
            allCavCmds{ix} = struct('vid', {}, 'v_next', {}, 'laneID', {}, 'lanePos_next', {}, 'x_target', {}, 'y_target', {}, 'u', {}, 'v', {}, 'distToStop', {});
        else
            % External leader for CAVs that are first on their lane: the vehicle
            % SUMO sees ahead of them beyond the stop line (inside the junction
            % or on the exit lane), used by the rear-end constraint only. A
            % leader that has just crossed at low speed would otherwise be
            % invisible to the approach controller.
            for vz = 1:numel(vehiclesInZone)
                vehiclesInZone(vz).extLeaderGap = NaN; vehiclesInZone(vz).extLeaderSpeed = NaN; vehiclesInZone(vz).extLeaderIsCAV = false;
            end
            for vz = 1:numel(vehiclesInZone)
                if ~strcmpi(vehiclesInZone(vz).typeID, 'CAV'), continue; end
                isFirst = true;
                for vz2 = 1:numel(vehiclesInZone)
                    if vz2 ~= vz && strcmp(vehiclesInZone(vz2).lane, vehiclesInZone(vz).lane) && vehiclesInZone(vz2).distToStop < vehiclesInZone(vz).distToStop
                        isFirst = false; break;
                    end
                end
                if ~isFirst, continue; end
                try
                    [lidE, gapE] = traci.vehicle.getLeader(vehiclesInZone(vz).id, 80);
                    if ~isempty(lidE) && isnumeric(gapE) && isfinite(gapE) && gapE >= 0
                        % Accept only a leader on this lane's own downstream path
                        % (its connections' internal lanes, their continuations and
                        % the exit edges). SUMO's getLeader also reports junction
                        % foes crossing in front, with the gap to the conflict point;
                        % those are the signal's business, not the rear-end constraint's.
                        leadLane = '';
                        for vo = 1:numel(allVehData)
                            if strcmp(allVehData(vo).id, char(lidE)), leadLane = allVehData(vo).lane; break; end
                        end
                        if on_downstream_path(vehiclesInZone(vz).id, vehiclesInZone(vz).lane, char(lidE), leadLane)
                            vehiclesInZone(vz).extLeaderGap = double(gapE);
                            vehiclesInZone(vz).extLeaderSpeed = traci.vehicle.getSpeed(char(lidE));
                            for vo = 1:numel(allVehData)
                                if strcmp(allVehData(vo).id, char(lidE)), vehiclesInZone(vz).extLeaderIsCAV = strcmpi(allVehData(vo).typeID, 'CAV'); break; end
                            end
                        end
                    end
                catch
                end
            end
            tPol = tic;
            cavCmds = cav_control_policy(vehiclesInZone, lanePhaseMap, phaseIdxSeq, durSeq, idm, cav, vehicleHeadway, cavHeadway, dt, tnow, yellowSeq);
            tp_ = toc(tPol); polTimeSum = polTimeSum + tp_; polTimeMax = max(polTimeMax, tp_); polCalls = polCalls + 1;
            allCavCmds{ix} = cavCmds;
        end

        % Crossed CAVs (inside the zone radius but no longer on an incoming
        % lane). Built every step, independently of vehiclesInZone.
        nAll = numel(allVehData);
        resetCmds = repmat(struct('vid', '', 'speed', 0), 1, nAll);
        nReset = 0;
        if ~isempty(posZ)
            for vi = 1:nAll
                vd = allVehData(vi);
                if strcmpi(vd.typeID, 'CAV') && ~isempty(vd.pos) && ~isempty(vd.lane)
                    dx = vd.pos(1) - posZ(1);
                    dy = vd.pos(2) - posZ(2);
                    dist = sqrt(dx*dx + dy*dy);
                    if dist < s.controlZoneRadius && ~isKey(lanePhaseMap, vd.lane)
                        nReset = nReset + 1;
                        resetCmds(nReset) = struct('vid', vd.id, 'speed', cav.v_max);
                    end
                end
            end
        end
        allResetCmds{ix} = resetCmds(1:nReset);

        % Save which phase is effectively green before processing transitions.
        prevEffGreenIdx = s.currentPhaseIdx;
        prevEffGreenDur = s.currentPhaseDur;

        % Advance signal state when current phase has expired.
        while tnow >= (s.currentPhaseStart + s.currentPhaseDur)
            s.phaseHistSum(s.currentPhaseIdx) = s.phaseHistSum(s.currentPhaseIdx) + s.currentPhaseDur;
            s.phaseHistCnt(s.currentPhaseIdx) = s.phaseHistCnt(s.currentPhaseIdx) + 1;

            s.currentPhaseIdx = mod(s.currentPhaseIdx, nP) + 1;
            s.currentPhaseStart = s.currentPhaseStart + s.currentPhaseDur;
            s.currentPhaseDur = s.futureDur(1);
            extPhase = mod(s.currentPhaseIdx - 1 + (2*nP - 1), nP) + 1;
            if s.phaseHistCnt(extPhase) > 0
                nomExt = s.phaseHistSum(extPhase) / s.phaseHistCnt(extPhase);
            else
                nomExt = s.phaseDurNom(extPhase);
            end
            s.futureDur = [s.futureDur(2:end), nomExt];

            tlsCmds{end+1} = s.phases{s.currentPhaseIdx}; %#ok<AGROW>

            fprintf('[%s] Switched to phase %d for %.2fs at t=%.2f\n', s.id, s.currentPhaseIdx, s.currentPhaseDur, tnow);

            if OPTIMIZE_SIGNALS
            vehiclesForOpt = filter_always_green_vehicles(vehiclesInZone, lanePhaseMap);
            if ~isempty(vehiclesForOpt)
                fixedDur = [s.currentPhaseDur, s.futureDur(1:nP-1)];
                fixedCounts = max(0, ceil(fixedDur ./ vehicleHeadway));

                schedulePhaseIdx = zeros(1, 2*nP);
                for i = 1:2*nP
                    schedulePhaseIdx(i) = mod(s.currentPhaseIdx - 1 + (i - 1), nP) + 1;
                end

                baselineTailCounts = max(0, ceil(s.futureDur(nP:2*nP-1) ./ vehicleHeadway));
                phaseVehMin = max(0, floor(s.phaseDurMin ./ vehicleHeadway));
                phaseVehMax = max(1, ceil(s.phaseDurMax ./ vehicleHeadway));

                fprintf('[%s] Optimization at phase start t=%.2f with %d vehicles (%d signal-dependent)\n', s.id, tnow, numel(vehiclesInZone), numel(vehiclesForOpt));

                tic
                [bestTailCounts, bestCost, optDetails] = optimize_signal_plan( ...
                    vehiclesForOpt, lanePhaseMap, schedulePhaseIdx, fixedCounts, ...
                    phaseVehMin, phaseVehMax, baselineTailCounts, ...
                    s.phaseHistSum ./ max(s.phaseHistCnt, 1), idm, vehicleHeadway, yellowDuration);
                optWall = toc;
                s.futureDur(nP:2*nP-1) = bestTailCounts .* vehicleHeadway;

                vehCountByPhase = count_vehicles_per_phase(vehiclesForOpt, lanePhaseMap, nP);
                for fi = 1:nP-1
                    futurePhase = mod(s.currentPhaseIdx - 1 + fi, nP) + 1;
                    if vehCountByPhase(futurePhase) == 0
                        s.futureDur(fi) = 0;
                    end
                end

                fprintf('[%s]   best cost=%.3f\n', s.id, bestCost);

                % ---- optimizer X-ray: counts per phase, in zone and on the whole approach ----
                stoppedInZone = vehiclesInZone([vehiclesInZone.speed] < 0.5);
                vehPP  = count_vehicles_per_phase(vehiclesInZone, lanePhaseMap, nP);
                stopPP = count_vehicles_per_phase(stoppedInZone, lanePhaseMap, nP);
                allOnApproach = struct('lane', {}, 'speed', {});
                for va = 1:numel(allVehData)
                    if isKey(lanePhaseMap, allVehData(va).lane)
                        allOnApproach(end+1) = struct('lane', allVehData(va).lane, 'speed', allVehData(va).speed); %#ok<AGROW>
                    end
                end
                vehPPall  = count_vehicles_per_phase(allOnApproach, lanePhaseMap, nP);
                stopPPall = count_vehicles_per_phase(allOnApproach([allOnApproach.speed] < 0.5), lanePhaseMap, nP);

                optLogIdx = optLogIdx + 1;
                optLog(optLogIdx) = struct('t', tnow, 'phaseIdx', s.currentPhaseIdx, ...
                    'nVehicles', numel(vehiclesForOpt), 'cost', bestCost, ...
                    'fixedDur', fixedDur, 'tailCounts', bestTailCounts, ...
                    'futureDur', s.futureDur(nP:2*nP-1), ...
                    'vehPerPhase', vehPP, 'stoppedPerPhase', stopPP, ...
                    'vehPerPhaseAll', vehPPall, 'stoppedPerPhaseAll', stopPPall, ...
                    'fixedCounts', fixedCounts, 'futureDurAfter', s.futureDur, ...
                    'currentPhaseDur', s.currentPhaseDur, 'searchMode', optDetails.searchMode, ...
                    'candidateCount', optDetails.candidateCount, 'step', optDetails.step, 'wallTime', optWall);
            end
            end % if OPTIMIZE_SIGNALS
        end

        % Clearance (yellow) transition: shown after every green of non-zero
        % duration whose next effective phase is a different phase. (The
        % network code showed it only when the green lasted longer than the
        % yellow itself, leaving short greens without any clearance.)
        if s.currentPhaseIdx ~= prevEffGreenIdx && prevEffGreenDur > 0
            tlsCmds{end+1} = s.yellowPhases{prevEffGreenIdx}; %#ok<AGROW>
            s.currentPhaseStart = s.currentPhaseStart + yellowDuration;
            s.yellowEndTime = s.currentPhaseStart;
            s.yellowFromPhase = prevEffGreenIdx;   % the phase whose clearance is being shown
        end

        allTlsCmds{ix} = tlsCmds;
        intx{ix} = s;
    end

    % =================================================================
    % PHASE 3: APPLY - execute all deferred TraCI commands
    % =================================================================
    % --- per-step CAV log (before APPLY so underCtrl reflects this step) ---
    cmdMap = containers.Map('KeyType','char','ValueType','any');
    for ix = 1:numIntx
        cmds_ = allCavCmds{ix};
        for ci = 1:numel(cmds_)
            cmdMap(cmds_(ci).vid) = cmds_(ci);
        end
    end
    try tlsNowStr = traci.trafficlights.getRedYellowGreenState(intx{1}.id); catch, tlsNowStr = ''; end
    for vi_log = 1:numel(allVehData)
        vd = allVehData(vi_log);
        if ~strcmpi(vd.typeID, 'CAV') || isempty(vd.lane), continue; end
        if stepLogN >= stepLogCap, break; end
        stepLogN = stepLogN + 1;
        stepLog.t(stepLogN)         = tnow;
        stepLog.vid{stepLogN}       = vd.id;
        stepLog.lane{stepLogN}      = vd.lane;
        stepLog.speed(stepLogN)     = vd.speed;
        stepLog.lanePos(stepLogN)   = vd.lanePos;
        stepLog.underCtrl(stepLogN) = isKey(cavUnderControl, vd.id);
        stepLog.tls{stepLogN}       = tlsNowStr;
        if isKey(cmdMap, vd.id)
            c_ = cmdMap(vd.id);
            stepLog.cmd_u(stepLogN)      = c_.u;
            stepLog.cmd_vnext(stepLogN)  = c_.v_next;
            stepLog.distToStop(stepLogN) = c_.distToStop;
            d_ = c_.diag;
            stepLog.t_target(stepLogN)   = d_.t_target;
            stepLog.u_nom(stepLogN)      = d_.u_nom;
            stepLog.cbf_active(stepLogN) = d_.cbf_active;
            stepLog.h(stepLogN)          = d_.h;
            stepLog.gap(stepLogN)        = d_.gap;
            stepLog.leader{stepLogN}     = d_.leader;
            stepLog.mustStop(stepLogN)   = d_.mustStop;
            stepLog.stopActive(stepLogN) = d_.stopActive;
            stepLog.greenNow(stepLogN)   = d_.greenNow;
            stepLog.greenEnd(stepLogN)   = d_.greenEnd;
            stepLog.profile{stepLogN}    = d_.profile;
            stepLog.standingStart(stepLogN) = d_.standingStart;
            stepLog.lineClamp(stepLogN)  = d_.lineClamp;
            stepLog.replanned(stepLogN)  = d_.replanned;
        end
    end

    % --- mid-junction stops: any vehicle at rest on an internal lane ---
    if mod(k, 5) == 0
        for vi_j = 1:numel(allVehData)
            vd = allVehData(vi_j);
            if isempty(vd.lane) || vd.lane(1) ~= ':' || vd.speed > 0.1, continue; end
            if jStopN >= 2000, break; end
            others = '';
            for vo = 1:numel(allVehData)
                if vo ~= vi_j && ~isempty(allVehData(vo).lane) && allVehData(vo).lane(1) == ':'
                    others = [others sprintf('%s@%s(%.1f) ', allVehData(vo).id, allVehData(vo).lane, allVehData(vo).speed)]; %#ok<AGROW>
                end
            end
            jStopN = jStopN + 1;
            jStop(jStopN) = struct('t', tnow, 'vid', vd.id, 'type', vd.typeID, 'lane', vd.lane, 'lanePos', vd.lanePos, 'tls', tlsNowStr, 'others', others);
        end
    end

    for ix = 1:numIntx
        s = intx{ix};
        cmds = allCavCmds{ix};

        switch APPLY_MODE
            case 'legacy_moveTo'
                % ---- Verbatim from FINAL\Final_play - Copy (5) ----
                % Moves each CAV twice per step (moveTo + SUMO's own step).
                for ci = 1:numel(cmds)
                    try
                        traci.vehicle.setSpeed(cmds(ci).vid, cmds(ci).v_next);
                    catch
                    end
                    try
                        traci.vehicle.moveTo(cmds(ci).vid, cmds(ci).laneID, cmds(ci).lanePos_next);
                    catch
                        try
                            traci.vehicle.moveToXY(cmds(ci).vid, '', 0, cmds(ci).x_target, cmds(ci).y_target, 0);
                        catch
                        end
                    end
                end

                resets = allResetCmds{ix};
                for ci = 1:numel(resets)
                    try
                        traci.vehicle.setSpeed(resets(ci).vid, resets(ci).speed);
                    catch
                    end
                end

            case 'fixed'
                % ---- Controller commands speed only ----
                % On first command inside the zone: disable SUMO's speed
                % checks (safe speed, accel/decel, right-of-way, red light,
                % speed limit -> speedMode 96) and freeze lane changes (512).
                % SUMO integrates position from the commanded speed
                % (ballistic update, see preliminaries.m).
                for ci = 1:numel(cmds)
                    vidc = cmds(ci).vid;
                    if ~isKey(cavUnderControl, vidc)
                        % speedMode 72 = bit3 (regard right-of-way when entering the
                        % junction, incl. vehicles still inside it) + bit6 (disregard
                        % speed limit); safe-speed, accel/decel bounds and red-light
                        % braking are off (the controller handles them). The
                        % right-of-way check is a backstop for the case the model does
                        % not cover: a vehicle (typically an HDV that entered late on
                        % yellow) stalled inside the junction on a conflicting path.
                        try traci.vehicle.setSpeedMode(vidc, 72); catch; end
                        try traci.vehicle.setLaneChangeMode(vidc, 512); catch; end
                        cavUnderControl(vidc) = true;
                    end
                    try
                        traci.vehicle.setSpeed(vidc, cmds(ci).v_next);
                    catch
                    end
                end

                % Crossed CAVs (junction-internal lane or outgoing edge, inside
                % the zone radius): DOWNSTREAM POLICY. Keep the CAV under the
                % controller with nominal u = u_max toward v_max and the SAME
                % rear-end constraint as on the approach (paper Eq. 18), with the
                % leader taken from SUMO's getLeader. By the time the CAV leaves
                % the radius it sits at gap s0 + v*T with matched speed, which is
                % also the equilibrium of the CAV vType's IDM (tau = T,
                % minGap = s0), so the hand-back below is smooth.
                resets = allResetCmds{ix};
                handled = containers.Map('KeyType','char','ValueType','logical');
                for ci = 1:numel(cmds)
                    handled(cmds(ci).vid) = true;
                end
                for ci = 1:numel(resets)
                    vidr = resets(ci).vid;
                    handled(vidr) = true;
                    if ~isKey(cavUnderControl, vidr)
                        continue;
                    end
                    vNow = NaN;
                    try vNow = traci.vehicle.getSpeed(vidr); catch; end
                    if isnan(vNow)
                        continue;
                    end
                    u_ds = cav.u_max;
                    % Leader: primary source = same-lane search in the vehicle
                    % data already queried this step (robust); secondary =
                    % SUMO's getLeader (also sees leaders across lane
                    % boundaries, e.g. internal lane -> outgoing edge).
                    [leadID, leadGap] = find_leader_same_lane(vidr, allVehData, laneVehIndex, idm.L);
                    try
                        % traci4matlab: [vehicleID, dist] = getLeader(vehID, lookahead)
                        [lid2, gap2] = traci.vehicle.getLeader(vidr, 200);
                        if ~isempty(lid2) && (ischar(lid2) || isstring(lid2)) && isnumeric(gap2) && isfinite(gap2) && gap2 >= 0 && gap2 < leadGap
                            leadID = char(lid2); leadGap = double(gap2);
                        end
                    catch
                    end
                    if ~isempty(leadID) && isfinite(leadGap)
                        vLead = vNow;
                        try vLead = traci.vehicle.getSpeed(leadID); catch; end
                        dv_ds = vLead - vNow;
                        % Comfort shaping of the NOMINAL behind an HDV leader (uncertain
                        % case): the same barrier form evaluated with a comfort headway
                        % T_c > T, bounded by a comfort deceleration, so the CAV eases off
                        % early instead of accelerating until the safety constraint bites.
                        % Not applied behind a CAV leader: the planned 0.5 s platoon
                        % headway is intended.
                        leadIsCAV = false;
                        for vo = 1:numel(allVehData)
                            if strcmp(allVehData(vo).id, leadID), leadIsCAV = strcmpi(allVehData(vo).typeID, 'CAV'); break; end
                        end
                        if ~leadIsCAV && isfield(cav, 'T_comfort')
                            Tc = cav.T_comfort;
                            h_c = leadGap - idm.s0 - vNow * Tc;
                            u_comf = min((dv_ds + cav.kappa * h_c) / Tc, (h_c + dv_ds * dt) / (Tc * dt + 0.5 * dt^2));
                            u_comf = max(u_comf, -1.0);   % comfort deceleration bound
                            u_ds = min(u_ds, u_comf);
                        end
                        % Safety constraint (paper Eq. 18) with T_safe, continuous + sampled-data form
                        h_ds = leadGap - idm.s0 - vNow * cav.T_safe; Teff_ds = cav.T_safe;
                        if strcmpi(cav.barrier, 'braking') && vNow > vLead
                            h_ds = h_ds - (vNow^2 - vLead^2) / (2 * abs(cav.u_min)); Teff_ds = cav.T_safe + vNow / abs(cav.u_min);
                        end
                        u_cbf_ds = min((dv_ds + cav.kappa * h_ds) / Teff_ds, (h_ds + dv_ds * dt) / (Teff_ds * dt + 0.5 * dt^2));
                        u_ds = min(u_ds, u_cbf_ds);
                        nDownLeader = nDownLeader + 1;
                    end
                    nDownCmds = nDownCmds + 1;
                    u_ds = min(max(u_ds, cav.u_min), cav.u_max);
                    vNext_ds = min(max(vNow + u_ds * dt, 0), cav.v_max);
                    try traci.vehicle.setSpeed(vidr, vNext_ds); catch; end
                    if dsLogN < stepLogCap
                        dsLogN = dsLogN + 1;
                        dsLog.t(dsLogN) = tnow; dsLog.vid{dsLogN} = vidr; dsLog.v(dsLogN) = vNow;
                        dsLog.gap(dsLogN) = leadGap; dsLog.u(dsLogN) = u_ds; dsLog.leader{dsLogN} = leadID;
                        try dsLog.lane{dsLogN} = traci.vehicle.getLaneID(vidr); catch, dsLog.lane{dsLogN} = ''; end
                    end
                end

                % Hand-back: CAVs still flagged as under control that received
                % no command this step have left the zone radius -> SUMO's
                % default checks and own speed.
                ctrlIDs = keys(cavUnderControl);
                for ci = 1:numel(ctrlIDs)
                    vidr = ctrlIDs{ci};
                    if isKey(handled, vidr)
                        continue;
                    end
                    try traci.vehicle.setSpeedMode(vidr, 31); catch; end
                    try traci.vehicle.setLaneChangeMode(vidr, 1621); catch; end
                    try traci.vehicle.setSpeed(vidr, -1); catch; end
                    remove(cavUnderControl, vidr);
                end

            otherwise
                error('Unknown APPLY_MODE "%s" (use ''fixed'' or ''legacy_moveTo'').', APPLY_MODE);
        end

        % Apply TLS state commands
        tlsCmds = allTlsCmds{ix};
        for ci = 1:numel(tlsCmds)
            try
                traci.trafficlights.setRedYellowGreenState(s.id, tlsCmds{ci});
            catch
            end
        end
    end
end

try
    traci.close();
catch ME
    warning('traci.close failed (%s) - SUMO had probably already exited; results are still saved.', ME.message);
end

% =========================================================================
% SAVE RESULTS AND PLOT
% =========================================================================
simData.trajLog     = trajLog;
simData.tlsLog      = tlsLog;
simData.tlsStates   = tlsStates;
simData.optLog      = optLog;
simData.pathDefs    = pathDefs;
simData.phases      = intx{1}.phases;
simData.tripRecords = tripRecords(1:nTrips);
simData.dt          = dt;
simData.controlZoneRadius = intx{1}.controlZoneRadius;
simData.APPLY_MODE  = APPLY_MODE;
simData.OPTIMIZE_SIGNALS = OPTIMIZE_SIGNALS;
simData.u_min       = cav.u_min;

% Trim and attach per-step CAV log
fn = fieldnames(stepLog);
for fi = 1:numel(fn)
    stepLog.(fn{fi}) = stepLog.(fn{fi})(1:stepLogN);
end
simData.stepLog = stepLog;
fn = fieldnames(dsLog);
for fi = 1:numel(fn)
    dsLog.(fn{fi}) = dsLog.(fn{fi})(1:dsLogN);
end
simData.dsLog = dsLog;
fn = fieldnames(qLog);
for fi = 1:numel(fn)
    qLog.(fn{fi}) = qLog.(fn{fi})(1:qLogN, :);
end
simData.qLog = qLog; simData.qLanes = qLanes;
simData.policyTime = [polTimeSum / max(polCalls, 1), polTimeMax, polCalls];
simData.jStop = jStop(1:jStopN);
simData.vehicleHeadway = vehicleHeadway;
simData.yellowDuration = yellowDuration;
simData.cav = cav; simData.idm = idm; simData.cavHeadway = cavHeadway;

matFileName = sprintf('sim_trajectory_data_%s.mat', APPLY_MODE);
save(matFileName, 'simData');
fprintf('Saved %d trajectory samples, %d TLS samples, %d optimization events, %d trips to %s\n', ...
    trajLogIdx, numel(tlsLog), optLogIdx, nTrips, matFileName);
fprintf('Downstream policy: %d commands, %d with a leader found\n', nDownCmds, nDownLeader);

% Time-space diagrams (adjust the window as needed)
plot_trajectories(simData, [0 min(120, simDuration_s)]);

% Kinematic consistency check: displacement rate vs. reported speed for
% CAVs inside the control zone (should be 1.0 if the controller's commands
% are what SUMO actually executes).
check_kinematics(simData);

% Collisions logged by SUMO during the run (collisions.xml in this folder).
% With APPLY_MODE = 'fixed' SUMO no longer protects CAVs, so any entry here
% is a real failure of the controller's own safety constraint.
pause(3);   % SUMO writes its output files at shutdown
report_collisions(fullfile(fileparts(mfilename('fullpath')), 'collisions.xml'));

% SUMO's own warnings involving CAVs (emergency braking, junction collisions)
sumoLogFile = fullfile(fileparts(mfilename('fullpath')), 'sumo_log.txt');
if exist(sumoLogFile, 'file')
    sl = strsplit(fileread(sumoLogFile), newline);
    wAll = sl(contains(sl, 'Warning'));
    wCav = wAll(contains(wAll, 'cav_'));
    fprintf('\n===== SUMO warnings: %d total, %d involving a CAV =====\n', numel(wAll), numel(wCav));
    for wi = 1:min(numel(wCav), 40), fprintf('%s\n', strtrim(wCav{wi})); end
    fprintf('=====================================================\n\n');
end

% Attribute every CAV speed drop larger than |u_min|*dt to its cause.
analyze_braking(simData);

% Behavioural X-ray of the controller (crossings on green, stops, launches,
% gaps, effort, hard braking).
audit_controller(simData);

clear global connections message

% =========================================================================
% LOCAL FUNCTIONS (verbatim from FINAL\Final_play - Copy (5)\run_from_matlab.m)
% =========================================================================
function tf = on_downstream_path(cavID, cavLane, leadID, leadLane)
% True if the leader is on the CAV's own path beyond the stop line:
%  - leader on an internal lane: its route contains the CAV's current edge
%    immediately followed by the CAV's next edge (same movement);
%  - leader on a normal lane: that lane's edge is the CAV's next edge.
% SUMO's getLeader also reports junction foes crossing in front (with the gap
% to the conflict point); those are the signal's business and are rejected
% here. Routes are fixed per vehicle and cached.
persistent routeCache
if isempty(routeCache), routeCache = containers.Map('KeyType', 'char', 'ValueType', 'any'); end
tf = false;
if isempty(leadLane) || isempty(cavLane) || cavLane(1) == ':', return; end
try
    if ~isKey(routeCache, cavID), routeCache(cavID) = traci.vehicle.getRoute(cavID); end
    rC = routeCache(cavID);
    curEdge = regexprep(cavLane, '_\d+$', '');
    ic = find(strcmp(rC, curEdge), 1);
    if isempty(ic) || ic == numel(rC), return; end
    nextEdge = rC{ic + 1};
    if leadLane(1) == ':'
        if ~isKey(routeCache, leadID), routeCache(leadID) = traci.vehicle.getRoute(leadID); end
        rL = routeCache(leadID);
        il = find(strcmp(rL, curEdge), 1);
        tf = ~isempty(il) && il < numel(rL) && strcmp(rL{il + 1}, nextEdge);
    else
        tf = strcmp(regexprep(leadLane, '_\d+$', ''), nextEdge);
    end
catch
    tf = false;
end
end

function [durSeq, yellowSeq] = build_signal_horizon(phaseIdxSeq, greenDurs, yellowDuration)
% Slot durations including clearance tails, and the per-slot tail vector.
% Tail after slot i iff greenDurs(i) > 0 and the next slot with non-zero
% duration has a different phase (a re-granted phase is continuous green).
% Slot 1 is the phase currently displayed: it gets its tail even when its
% remaining green is exactly 0 (the step at which it expires, before the
% TLS switch is processed), otherwise the CAVs would see the next phase
% starting now instead of after the clearance.
% The last non-zero slot gets a tail (its successor is unknown).
n = numel(greenDurs);
yellowSeq = zeros(1, n);
for i = 1:n
    if greenDurs(i) <= 0 && i > 1, continue; end
    nextPhase = [];
    for j = i+1:n
        if greenDurs(j) > 0, nextPhase = phaseIdxSeq(j); break; end
    end
    if isempty(nextPhase) || nextPhase ~= phaseIdxSeq(i)
        yellowSeq(i) = yellowDuration;
    end
end
durSeq = greenDurs + yellowSeq;
end

function [leadID, gap] = find_leader_same_lane(vid, allVehData, laneVehIndex, vehLength)
% Leader on the same lane from the per-step vehicle data: the vehicle with the
% smallest lane position greater than ours. gap is bumper-to-bumper.
leadID = ''; gap = Inf;
me = [];
for i = 1:numel(allVehData)
    if strcmp(allVehData(i).id, vid), me = allVehData(i); break; end
end
if isempty(me) || isempty(me.lane) || ~isKey(laneVehIndex, me.lane), return; end
cand = laneVehIndex(me.lane);
best = Inf;
for k = 1:numel(cand)
    o = allVehData(cand(k));
    if strcmp(o.id, vid), continue; end
    d = o.lanePos - me.lanePos;
    if d > 0 && d < best
        best = d; leadID = o.id;
    end
end
if ~isempty(leadID)
    gap = max(best - vehLength, 0);
end
end
function [vehiclesInZone, lanePhaseMap, incomingOutEdges, incomingConnLanes] = collect_control_zone_state(tlsID, range_m, phases, allVehData, laneVehIndex)

vehiclesInZone = struct('id', {}, 'lane', {}, 'lanePos', {}, 'speed', {}, 'distToStop', {}, 'typeID', {}, 'pos', {});
lanePhaseMap   = containers.Map('KeyType', 'char', 'ValueType', 'any');

incomingOutEdges  = containers.Map('KeyType', 'char', 'ValueType', 'any');
incomingConnLanes = containers.Map('KeyType', 'char', 'ValueType', 'any');

persistent tlsCache
if isempty(tlsCache)
    tlsCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
end

try
    posZ = traci.junction.getPosition(tlsID);
catch
    posZ = [];
end

if isempty(posZ)
    return;
end

if isKey(tlsCache, tlsID)
    cached = tlsCache(tlsID);
    if isequal(cached.phases, phases)
        lanePhaseMap = cached.lanePhaseMap;
        incomingLanes = cached.incomingLanes;
        incomingOutEdges = cached.outEdges;
        incomingConnLanes = cached.connLanes;
    else
        incomingLanes = {};
    end
else
    incomingLanes = {};
end

if isempty(incomingLanes)

    try
        controlledLinks = traci.trafficlights.getControlledLinks(tlsID);
        for signalIdx = 1:numel(controlledLinks)
            signalGroup = controlledLinks{signalIdx};
            if isempty(signalGroup)
                continue;
            end

            if iscell(signalGroup) && numel(signalGroup) >= 3 && ischar(signalGroup{1})
                connectionList = {signalGroup};
            elseif iscell(signalGroup)
                connectionList = signalGroup;
            else
                continue;
            end

            for connIdx = 1:numel(connectionList)
                conn = connectionList{connIdx};
                if ~iscell(conn) || numel(conn) < 1
                    continue;
                end

                inLane = '';
                outLane = '';
                viaLane = '';

                if numel(conn) >= 1 && (ischar(conn{1}) || isstring(conn{1}))
                    inLane = char(conn{1});
                end
                if numel(conn) >= 2 && (ischar(conn{2}) || isstring(conn{2}))
                    outLane = char(conn{2});
                end
                if numel(conn) >= 3 && (ischar(conn{3}) || isstring(conn{3}))
                    viaLane = char(conn{3});
                end

                if isempty(inLane)
                    continue;
                end

                if ~ismember(inLane, incomingLanes)
                    incomingLanes{end+1} = inLane; %#ok<AGROW>
                end

                if ~isempty(outLane)
                    try
                        outEdge = traci.lane.getEdgeID(outLane);
                        if ~isKey(incomingOutEdges, inLane)
                            incomingOutEdges(inLane) = {outEdge};
                        else
                            incomingOutEdges(inLane) = unique([incomingOutEdges(inLane), {outEdge}]);
                        end
                    catch
                    end
                end

                if ~isempty(viaLane)
                    if ~isKey(incomingConnLanes, inLane)
                        incomingConnLanes(inLane) = {viaLane};
                    else
                        incomingConnLanes(inLane) = unique([incomingConnLanes(inLane), {viaLane}]);
                    end
                end

                if ~isKey(lanePhaseMap, inLane)
                    lanePhaseMap(inLane) = false(1, numel(phases));
                end

                greenMask = lanePhaseMap(inLane);
                for phaseIdx = 1:numel(phases)
                    phaseState = phases{phaseIdx};
                    if signalIdx <= length(phaseState)
                        signalChar = phaseState(signalIdx);
                        if signalChar == 'G' || signalChar == 'g'
                            greenMask(phaseIdx) = true;
                        end
                    end
                end
                lanePhaseMap(inLane) = greenMask;
            end
        end

        tlsCache(tlsID) = struct( ...
            'phases', {phases}, ...
            'lanePhaseMap', lanePhaseMap, ...
            'incomingLanes', {incomingLanes}, ...
            'outEdges', incomingOutEdges, ...
            'connLanes', incomingConnLanes);
    catch
        incomingLanes = {};
    end
end

if isempty(incomingLanes)
    return;
end

% Fast path: use pre-queried vehicle data if available
if nargin >= 4 && ~isempty(allVehData)
    if nargin >= 5 && ~isempty(laneVehIndex)
        candidateIdx = [];
        for li = 1:numel(incomingLanes)
            if isKey(laneVehIndex, incomingLanes{li})
                candidateIdx = [candidateIdx, laneVehIndex(incomingLanes{li})]; %#ok<AGROW>
            end
        end
    else
        candidateIdx = 1:numel(allVehData);
    end
    nCand = numel(candidateIdx);
    if nCand == 0
        return;
    end
    vehiclesInZone(nCand) = struct('id', '', 'lane', '', 'lanePos', 0, 'speed', 0, 'distToStop', 0, 'typeID', '', 'pos', []);
    nFound = 0;
    for ci = 1:nCand
        vd = allVehData(candidateIdx(ci));
        laneID = vd.lane;
        if isempty(laneID)
            continue;
        end
        posV = vd.pos;
        if isempty(posV)
            continue;
        end
        dx = posV(1) - posZ(1);
        dy = posV(2) - posZ(2);
        dist = sqrt(dx*dx + dy*dy);
        if dist > range_m
            continue;
        end
        lanePos = vd.lanePos;
        speed = vd.speed;
        if isnan(lanePos) || isnan(speed)
            continue;
        end
        distToStop = compute_dist_to_stop_cached(laneID, lanePos);
        if isnan(distToStop)
            continue;
        end
        nFound = nFound + 1;
        vehiclesInZone(nFound) = struct('id', vd.id, 'lane', laneID, ...
            'lanePos', lanePos, 'speed', speed, 'distToStop', distToStop, ...
            'typeID', vd.typeID, 'pos', posV);
    end
    vehiclesInZone = vehiclesInZone(1:nFound);
    return;
end

vehIDs = traci.vehicle.getIDList();
for i = 1:numel(vehIDs)
    vid = vehIDs{i};

    try
        laneID = traci.vehicle.getLaneID(vid);
    catch
        laneID = '';
    end
    if isempty(laneID)
        continue;
    end
    if ~any(strcmp(laneID, incomingLanes))
        continue;
    end

    try
        posV = traci.vehicle.getPosition(vid);
    catch
        posV = [];
    end
    if isempty(posV)
        continue;
    end

    dx = posV(1) - posZ(1);
    dy = posV(2) - posZ(2);
    dist = sqrt(dx * dx + dy * dy);
    if dist > range_m
        continue;
    end

    try
        lanePos = traci.vehicle.getLanePosition(vid);
    catch
        lanePos = NaN;
    end
    try
        speed = traci.vehicle.getSpeed(vid);
    catch
        speed = NaN;
    end

    distToStop = NaN;
    if ~isnan(lanePos)
        try
            laneLength = traci.lane.getLength(laneID);
        catch
            laneLength = NaN;
        end
        if ~isnan(laneLength)
            distToStop = max(laneLength - lanePos, 0);
        end
    end
    if isnan(distToStop) || isnan(speed)
        continue;
    end

    try
        typeID = traci.vehicle.getTypeID(vid);
    catch
        typeID = 'HDV';
    end

    vehiclesInZone(end+1) = struct( ...
        'id', vid, ...
        'lane', laneID, ...
        'lanePos', lanePos, ...
        'speed', speed, ...
        'distToStop', distToStop, ...
        'typeID', typeID, ...
        'pos', posV); %#ok<AGROW>
end

end

function vehCount = count_vehicles_per_phase(vehiclesInZone, lanePhaseMap, nPhases)
% Count vehicles served by each phase based on lane-phase mapping.
vehCount = zeros(1, nPhases);
for vi = 1:numel(vehiclesInZone)
    lid = vehiclesInZone(vi).lane;
    if isKey(lanePhaseMap, lid)
        gv = lanePhaseMap(lid);
        for ph = 1:numel(gv)
            if gv(ph)
                vehCount(ph) = vehCount(ph) + 1;
            end
        end
    end
end
end

function filtered = filter_always_green_vehicles(vehiclesInZone, lanePhaseMap)
% Remove vehicles on lanes that are green in ALL phases (always-green).
if isempty(vehiclesInZone)
    filtered = vehiclesInZone;
    return;
end
keep = true(1, numel(vehiclesInZone));
for vi = 1:numel(vehiclesInZone)
    lid = vehiclesInZone(vi).lane;
    if isKey(lanePhaseMap, lid)
        gv = lanePhaseMap(lid);
        if all(gv)
            keep(vi) = false;
        end
    end
end
filtered = vehiclesInZone(keep);
end

function allVehData = batch_query_all_vehicles()
% Query all vehicle state from SUMO in a single pass using subscriptions.
import traci.constants

vehIDs = traci.vehicle.getIDList();
n = numel(vehIDs);
if n == 0
    allVehData = struct('id', {}, 'lane', {}, 'pos', {}, 'speed', {}, 'lanePos', {}, 'typeID', {});
    return;
end

ID_SPEED    = constants.VAR_SPEED;
ID_POSITION = constants.VAR_POSITION;
ID_LANE_ID  = constants.VAR_LANE_ID;
ID_LANE_POS = constants.VAR_LANEPOSITION;
ID_TYPE     = constants.VAR_TYPE;

subMap = containers.Map();
try
    res = traci.vehicle.getSubscriptionResults();
    if isa(res, 'containers.Map')
        subMap = res;
    end
catch
end

allVehData = repmat(struct('id', '', 'lane', '', 'pos', [], 'speed', NaN, 'lanePos', NaN, 'typeID', ''), 1, n);
validCount = 0;
needSubscribe = cell(1, 0);

for i = 1:n
    vid = vehIDs{i};
    gotData = false;

    if isKey(subMap, vid)
        try
            vres    = subMap(vid);
            laneID  = vres(ID_LANE_ID);
            posV    = vres(ID_POSITION);
            speed   = vres(ID_SPEED);
            lanePos = vres(ID_LANE_POS);
            typeID  = vres(ID_TYPE);
            gotData = true;
        catch
        end
    end

    if ~gotData
        try
            laneID  = traci.vehicle.getLaneID(vid);
            posV    = traci.vehicle.getPosition(vid);
            speed   = traci.vehicle.getSpeed(vid);
            lanePos = traci.vehicle.getLanePosition(vid);
            typeID  = traci.vehicle.getTypeID(vid);
        catch
            continue;
        end
        needSubscribe{end+1} = vid; %#ok<AGROW>
    end

    validCount = validCount + 1;
    allVehData(validCount) = struct('id', vid, 'lane', laneID, 'pos', posV, ...
        'speed', speed, 'lanePos', lanePos, 'typeID', typeID);
end
allVehData = allVehData(1:validCount);

if ~isempty(needSubscribe)
    subVars = {ID_SPEED, ID_POSITION, ID_LANE_ID, ID_LANE_POS, ID_TYPE};
    for i = 1:numel(needSubscribe)
        try
            traci.vehicle.subscribe(needSubscribe{i}, subVars);
        catch
        end
    end
end
end

function laneVehIndex = build_lane_vehicle_index(allVehData)
% Map from lane ID to indices of vehicles on that lane.
laneVehIndex = containers.Map('KeyType', 'char', 'ValueType', 'any');
for i = 1:numel(allVehData)
    lid = allVehData(i).lane;
    if isempty(lid)
        continue;
    end
    if isKey(laneVehIndex, lid)
        laneVehIndex(lid) = [laneVehIndex(lid), i];
    else
        laneVehIndex(lid) = i;
    end
end
end

function distToStop = compute_dist_to_stop_cached(laneID, lanePos)
% Distance to stop line with cached lane lengths (lane.getLength, not edge).
persistent laneLenCache
if isempty(laneLenCache)
    laneLenCache = containers.Map('KeyType', 'char', 'ValueType', 'double');
end
distToStop = NaN;
try
    if isKey(laneLenCache, laneID)
        laneLength = laneLenCache(laneID);
    else
        laneLength = traci.lane.getLength(laneID);
        laneLenCache(laneID) = laneLength;
    end
    distToStop = max(laneLength - lanePos, 0);
catch
    return;
end
end
