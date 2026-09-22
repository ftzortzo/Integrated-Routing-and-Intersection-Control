function stats = check_kinematics(simData)
% CHECK_KINEMATICS  Compare each vehicle's actual displacement rate with the
%   speed SUMO reports for it, inside the control zone.
%
%   For every logged vehicle, consecutive samples on the approach
%   (dist < 0) are used to compute
%       ratio = (dist(k+1) - dist(k)) / (t(k+1) - t(k))  /  mean(speed(k), speed(k+1))
%   If the vehicle's motion is consistent with its reported speed the ratio
%   is 1.0 (HDVs always are). A ratio of 2.0 for CAVs means the vehicle is
%   being moved twice per step (setSpeed + moveTo), i.e. the controller's
%   commands are NOT what SUMO executes.
%
%   stats = check_kinematics(simData)
%   stats = check_kinematics()            % loads sim_trajectory_data_fixed.mat

if nargin < 1 || isempty(simData)
    S = load('sim_trajectory_data_fixed.mat');
    simData = S.simData;
end

trajLog = simData.trajLog;
if isempty(trajLog)
    warning('No trajectory data.');
    stats = [];
    return;
end
if isfield(simData, 'controlZoneRadius')
    R = simData.controlZoneRadius;
else
    R = 200;
end

allT    = [trajLog.t];
allDist = [trajLog.dist];
allSpd  = [trajLog.speed];
allVid  = {trajLog.vid};
allType = {trajLog.typeID};

vids = unique(allVid);
stats = struct('vid', {}, 'type', {}, 'nSamples', {}, 'medianRatio', {}, 'maxRatio', {});

fprintf('\n===== Kinematic consistency check (inside control zone, %d m) =====\n', R);
fprintf('%-14s %-5s %8s %12s %10s\n', 'vehicle', 'type', 'samples', 'medianRatio', 'maxRatio');

for vi = 1:numel(vids)
    m = strcmp(allVid, vids{vi});
    t = allT(m); d = allDist(m); s = allSpd(m); ty = allType(m);
    [t, o] = sort(t); d = d(o); s = s(o);

    % approach samples inside the zone only
    inZone = d < 0 & d > -R;
    t = t(inZone); d = d(inZone); s = s(inZone);
    if numel(t) < 4, continue; end

    dd = diff(d) ./ diff(t);
    sm = (s(1:end-1) + s(2:end)) / 2;
    ok = sm > 1.0;                 % ignore near-standstill samples
    if nnz(ok) < 3, continue; end
    r = dd(ok) ./ sm(ok);

    stats(end+1) = struct('vid', vids{vi}, 'type', ty{1}, 'nSamples', nnz(ok), ...
        'medianRatio', median(r), 'maxRatio', max(r)); %#ok<AGROW>
    fprintf('%-14s %-5s %8d %12.3f %10.3f\n', vids{vi}, ty{1}, nnz(ok), median(r), max(r));
end

isCAV = strcmpi({stats.type}, 'CAV');
if any(isCAV)
    fprintf('\nCAV median ratio (all CAVs): %.3f\n', median([stats(isCAV).medianRatio]));
end
if any(~isCAV)
    fprintf('HDV median ratio (all HDVs): %.3f\n', median([stats(~isCAV).medianRatio]));
end
fprintf('Expected if controller commands are executed as intended: 1.000\n');
fprintf('======================================================================\n\n');
end
