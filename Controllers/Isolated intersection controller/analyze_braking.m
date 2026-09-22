function events = analyze_braking(simData, threshFactor)
% ANALYZE_BRAKING  Attribute every large one-step speed drop of a CAV to a cause.
%
%   events = analyze_braking(simData)
%   events = analyze_braking(simData, threshFactor)   % default 1.5
%
%   A "large" drop is dv < -threshFactor * |u_min| * dt in a single step
%   (with u_min = -3, dt = 0.1: more than 0.45 m/s per step, i.e. > 4.5 m/s^2).
%   Each event is classified from the per-step log:
%     'controller'   the drop equals the controller's own commanded v_next
%                    (the controller asked for it -> look at cmd_u)
%     'sumo-override' CAV was under control but SUMO applied a different speed
%     'hand-back'    first step after control ended (SUMO's IDM took over)
%     'sumo-driven'  CAV was not under control (before the zone / downstream)
%     'unknown'      none of the above

if nargin < 2 || isempty(threshFactor), threshFactor = 1.5; end
if ~isfield(simData, 'stepLog') || isempty(simData.stepLog.t)
    fprintf('\n===== analyze_braking: no per-step log in simData =====\n\n');
    events = [];
    return;
end
L  = simData.stepLog;
dt = simData.dt;
if isfield(simData, 'u_min'), u_min = simData.u_min; else, u_min = -3; end
thresh = threshFactor * abs(u_min) * dt;

events = struct('t', {}, 'vid', {}, 'lane', {}, 'v_prev', {}, 'v', {}, ...
    'decel', {}, 'cause', {}, 'cmd_u', {}, 'cmd_vnext', {}, 'distToStop', {});

vids = unique(L.vid);
for k = 1:numel(vids)
    m = find(strcmp(L.vid, vids{k}));
    [~, o] = sort(L.t(m)); m = m(o);
    for j = 2:numel(m)
        i0 = m(j-1); i1 = m(j);
        dv = L.speed(i1) - L.speed(i0);
        if dv >= -thresh, continue; end

        cmdV = L.cmd_vnext(i0);
        if ~isnan(cmdV) && abs(cmdV - L.speed(i1)) < 1e-3
            cause = 'controller';
        elseif ~isnan(cmdV)
            cause = 'sumo-override';
        elseif L.underCtrl(i0) && ~L.underCtrl(i1)
            cause = 'hand-back';
        elseif ~L.underCtrl(i0)
            cause = 'sumo-driven';
        else
            cause = 'unknown';
        end

        events(end+1) = struct('t', L.t(i1), 'vid', vids{k}, 'lane', L.lane{i1}, ...
            'v_prev', L.speed(i0), 'v', L.speed(i1), 'decel', -dv/dt, 'cause', cause, ...
            'cmd_u', L.cmd_u(i0), 'cmd_vnext', cmdV, 'distToStop', L.distToStop(i0)); %#ok<AGROW>
    end
end

fprintf('\n===== CAV speed drops > %.2f m/s^2 in one step: %d events =====\n', thresh/dt, numel(events));
if ~isempty(events)
    [~, o] = sort([events.t]); events = events(o);
    fprintf('%-7s %-12s %-8s %7s %7s %8s %-14s %8s %9s %9s\n', ...
        'time', 'vehicle', 'lane', 'v_prev', 'v', 'decel', 'cause', 'cmd_u', 'cmd_vnext', 'distStop');
    for e = 1:min(numel(events), 60)
        ev = events(e);
        fprintf('%-7.1f %-12s %-8s %7.2f %7.2f %8.2f %-14s %8.2f %9.2f %9.2f\n', ...
            ev.t, ev.vid, ev.lane, ev.v_prev, ev.v, ev.decel, ev.cause, ev.cmd_u, ev.cmd_vnext, ev.distToStop);
    end
    if numel(events) > 60, fprintf('... (%d more)\n', numel(events) - 60); end
    causes = {events.cause};
    uc = unique(causes);
    fprintf('\nBy cause: ');
    for c = 1:numel(uc)
        fprintf('%s = %d   ', uc{c}, nnz(strcmp(causes, uc{c})));
    end
    fprintf('\n');
end
fprintf('=================================================================\n\n');
end
