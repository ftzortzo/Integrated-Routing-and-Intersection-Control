function res = run_scenario(name, flows, opts)
% RUN_SCENARIO  Write a route file for a scenario, run the pipeline, collect the audit.
%
%   res = run_scenario(name, flows, opts)
%   flows: struct array with fields route ('NE','NS','NW','EN','ES','EW','SN','SE','SW','WN','WE','WS'),
%          vph (total veh/h on that movement), pen (CAV fraction 0..1)
%   opts:  duration (s, default 300), seed (default 42), departSpeed ('0'|'max'|'random',
%          default '0'), departLane ('best'|'first', default 'best')
%
% Restores the original quickstart.rou.xml afterwards.

if nargin < 3, opts = struct(); end
if ~isfield(opts, 'duration'),    opts.duration = 300; end
if ~isfield(opts, 'seed'),        opts.seed = 42; end
if ~isfield(opts, 'departSpeed'), opts.departSpeed = '0'; end
if ~isfield(opts, 'departLane'),  opts.departLane = 'best'; end

here = fileparts(mfilename('fullpath'));
rouFile = fullfile(here, 'quickstart.rou.xml');
backup  = fullfile(here, 'quickstart.rou.xml.bak');
copyfile(rouFile, backup);

edges = containers.Map({'N','E','S','W'}, {'NC','EC','SC','WC'});
outE  = containers.Map({'N','E','S','W'}, {'CN','CE','CS','CW'});
txt = fileread(rouFile);
i0 = strfind(txt, '<route id=');
head = txt(1:i0(1)-1);          % keep vType block
% Optional overrides of the CAV vType (kept consistent with the controller)
if isfield(opts, 'decel'),  head = regexprep(head, '(<vType id="CAV"[^>]*decel=")[^"]*(")', sprintf('$1%g$2', opts.decel)); head = regexprep(head, '(<vType id="CAV"[^>]*emergencyDecel=")[^"]*(")', sprintf('$1%g$2', opts.decel)); end
if isfield(opts, 'T_safe'), head = regexprep(head, '(<vType id="CAV"[^>]*tau=")[^"]*(")', sprintf('$1%g$2', opts.T_safe)); end
body = '';
for r = {'NE','NS','NW','EN','ES','EW','SN','SE','SW','WN','WE','WS'}
    body = [body sprintf('  <route id="route%s" edges="%s %s"/>\n', r{1}, edges(r{1}(1)), outE(r{1}(2)))]; %#ok<AGROW>
end
body = [body sprintf('\n  <!-- scenario: %s -->\n', name)];
for i = 1:numel(flows)
    f = flows(i);
    vc = round(f.vph * f.pen); vh = round(f.vph * (1 - f.pen));
    if vh > 0
        body = [body sprintf('  <flow id="hdv_%s" type="HDV" route="route%s" begin="0" end="3600" vehsPerHour="%d" departLane="%s" departSpeed="%s"/>\n', lower(f.route), f.route, vh, opts.departLane, opts.departSpeed)]; %#ok<AGROW>
    end
    if vc > 0
        body = [body sprintf('  <flow id="cav_%s" type="CAV" route="route%s" begin="0" end="3600" vehsPerHour="%d" departLane="%s" departSpeed="%s"/>\n', lower(f.route), f.route, vc, opts.departLane, opts.departSpeed)]; %#ok<AGROW>
    end
end
fid = fopen(rouFile, 'w'); fwrite(fid, [head body sprintf('</routes>\n')]); fclose(fid);

res = struct('name', name, 'ok', false);
try
    setenv('SUMO_HOME', 'C:\Program Files (x86)\Eclipse\Sumo');
    addpath(genpath('C:\Program Files\MATLAB\traci4matlab')); rehash;
    clear global connections message
    SIM_DURATION_OVERRIDE = opts.duration; %#ok<NASGU>
    SUMO_SEED = opts.seed; %#ok<NASGU>
    if isfield(opts, 'green_margin'), GREEN_MARGIN_OVERRIDE = opts.green_margin; end %#ok<NASGU>
    if isfield(opts, 'kappa'), KAPPA_OVERRIDE = opts.kappa; end %#ok<NASGU>
    if isfield(opts, 'barrier'), BARRIER_OVERRIDE = opts.barrier; end %#ok<NASGU>
    if isfield(opts, 'leaderPred'), LEADERPRED_OVERRIDE = opts.leaderPred; end %#ok<NASGU>
    if isfield(opts, 'T_safe'), TSAFE_OVERRIDE = opts.T_safe; end %#ok<NASGU>
    if isfield(opts, 'n_max'), N_MAX_OVERRIDE = opts.n_max; end %#ok<NASGU>
    if isfield(opts, 'u_min'), UMIN_OVERRIDE = opts.u_min; end %#ok<NASGU>
    logTxt = evalc('run_from_matlab');
    close all;

    % ---- collect ----
    res.ok = true;
    res.trips = simData.tripRecords;
    tt = [res.trips.arrive] - [res.trips.depart]; ty = {res.trips.type};
    res.tripCAV = mean(tt(strcmp(ty, 'CAV'))); res.tripHDV = mean(tt(strcmp(ty, 'HDV')));
    res.nCAV = nnz(strcmp(ty, 'CAV')); res.nHDV = nnz(strcmp(ty, 'HDV'));
    c = fileread(fullfile(here, 'collisions.xml')); res.collisions = numel(strfind(c, '<collision '));
    sl = strsplit(fileread(fullfile(here, 'sumo_log.txt')), newline);
    w = sl(contains(sl, 'Warning')); res.warnAll = numel(w); res.warnCAV = nnz(contains(w, 'cav_'));
    res.warnings = w;
    res.cannotStop = nnz(contains(strsplit(logTxt, newline), 'cannot stop'));
    A = evalc('R = audit_controller(simData);');
    res.audit = R; res.auditText = A;
    L = simData.stepLog; cmd = ~isnan(L.cmd_u);
    res.ratioCAV = median_ratio(simData);
    res.minH = R.min_h; res.violations = R.violations; res.wasted = R.wastedGreens; res.nStops = R.nStops;
    res.uMinSteps = nnz(L.cmd_u <= -2.999); res.cmdSteps = nnz(cmd);
    Q = simData.qLog; res.maxQueue = max(Q.stopped, [], 1); res.maxOnLane = max(Q.total, [], 1);
    res.nOpt = numel(simData.optLog);
    if isfield(simData, 'policyTime'), res.polMean = simData.policyTime(1); res.polMax = simData.policyTime(2); end
    if res.nOpt > 0 && isfield(simData.optLog, 'wallTime')
        wt = [simData.optLog.wallTime]; res.optWallMean = mean(wt); res.optWallMax = max(wt);
        cc = [simData.optLog.candidateCount]; res.candMax = max(cc); res.coarseCalls = nnz(contains({simData.optLog.searchMode}, 'coarse'));
    end
    res.log = logTxt;
    safeName = regexprep(name, '[^A-Za-z0-9]+', '_');
    save(fullfile(here, ['sim_' safeName '.mat']), 'simData');
    res.simFile = ['sim_' safeName '.mat'];
catch ME
    res.err = ME.message;
    res.log = '';
end
system('taskkill /IM sumo-gui.exe /F >nul 2>&1');
copyfile(backup, rouFile); delete(backup);

if res.ok
    fprintf('\n=== %s: trips CAV %.1f / HDV %.1f (n=%d/%d) | collisions %d | SUMO warn %d (CAV %d) | red-light violations %d | wasted greens %d | cannot-stop %d | min h %.2f | ratio %.3f | u_min steps %d/%d | max queue %s\n', ...
        name, res.tripCAV, res.tripHDV, res.nCAV, res.nHDV, res.collisions, res.warnAll, res.warnCAV, res.violations, res.wasted, res.cannotStop, res.minH, res.ratioCAV, res.uMinSteps, res.cmdSteps, mat2str(res.maxQueue));
else
    fprintf('\n=== %s: FAILED: %s\n', name, res.err);
end
end

function r = median_ratio(simData)
% Displacement/speed ratio for CAVs while commanded (from the per-step log,
% lane position), so it works for any route.
L = simData.stepLog; r = NaN;
t = L.t(:); vid = L.vid(:); lane = L.lane(:); lp = L.lanePos(:); v = L.speed(:); cmd = ~isnan(L.cmd_u(:));
vids = unique(vid); rs = [];
for k = 1:numel(vids)
    idx = find(strcmp(vid, vids{k}) & cmd); [~, o] = sort(t(idx)); idx = idx(o);
    if numel(idx) < 6, continue; end
    rr = [];
    for j = 2:numel(idx)
        a = idx(j-1); b = idx(j);
        if ~strcmp(lane{a}, lane{b}) || t(b) - t(a) > 0.15, continue; end
        sm = (v(a) + v(b)) / 2;
        if sm > 1, rr(end+1) = ((lp(b) - lp(a)) / (t(b) - t(a))) / sm; end %#ok<AGROW>
    end
    if numel(rr) >= 5, rs(end+1) = median(rr); end %#ok<AGROW>
end
if ~isempty(rs), r = median(rs); end
end
