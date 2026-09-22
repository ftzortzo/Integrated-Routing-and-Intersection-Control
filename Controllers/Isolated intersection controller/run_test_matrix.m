function run_test_matrix(outCsv, sel)
% RUN_TEST_MATRIX  Robustness campaign for the isolated-intersection controller.
%
%   run_test_matrix()                  runs the whole matrix, writes test_matrix.csv
%   run_test_matrix('my.csv')          custom output file
%   run_test_matrix('my.csv', 1:5)     only scenarios 1..5 of the list
%
% Each run: 300 s, one CSV row with the audit metrics, full log in sim_<name>.mat.
% Acceptance per run: collisions = 0, red-light entries = 0, wasted greens = 0,
% mid-junction stops = 0, min h >= -0.1 m, kinematic ratio = 1.000,
% optimizer max time < 1 s, CAV trips not worse than HDV trips by > 5 s.
%
% Launch in a MATLAB instance started in this folder with SUMO_HOME set
% (run_scenario sets it) and leave it; ~4-5 min per run.

if nargin < 1 || isempty(outCsv), outCsv = 'test_matrix.csv'; end
here = fileparts(mfilename('fullpath')); cd(here);

% ---- demand patterns (veh/h per movement) at three levels ----
levels = struct('name', {'light','moderate','heavy'}, 'scale', {0.4, 0.8, 1.2});
patterns = {};
patterns{end+1} = struct('name','straight4', 'routes',{{'NS','SN','EW','WE'}}, 'vph',[600 600 600 600]);
patterns{end+1} = struct('name','turnmix',   'routes',{{'NS','NE','NW','SN','SE','SW','EW','WE'}}, 'vph',[400 150 150 400 150 150 300 300]);
patterns{end+1} = struct('name','oneheavy',  'routes',{{'NE','NS','SN','EW','WE','ES'}}, 'vph',[700 150 150 150 150 150]);
patterns{end+1} = struct('name','all12',     'routes',{{'NE','NS','NW','EN','ES','EW','SN','SE','SW','WN','WE','WS'}}, 'vph',120*ones(1,12));
pens  = [0.1 0.5 0.9 1.0];
seeds = [101 202];

% ---- build the list ----
list = {};
for p = 1:numel(patterns)
    for l = 1:numel(levels)
        for pen = pens
            for sd = seeds
                if l == 1 && sd == seeds(2), continue; end            % light: one seed
                if pen == 0.9 && l ~= 2, continue; end                  % 0.9 only at moderate
                dsp = '0'; if mod(sd, 2) == 0, dsp = 'max'; end          % alternate departure speed
                list{end+1} = struct('pattern', patterns{p}, 'level', levels(l), 'pen', pen, 'seed', sd, 'departSpeed', dsp); %#ok<AGROW>
            end
        end
    end
end
if nargin < 2 || isempty(sel), sel = 1:numel(list); end
fprintf('%d scenarios in the matrix, running %d\n', numel(list), numel(sel));

hdr = 'idx,name,pattern,level,pen,seed,departSpeed,nCAV,nHDV,tripCAV,tripHDV,collisions,warnCAV,redViol,yellowCross,wasted,jStops,cannotStop,minH,cutins,ratio,uMinSteps,cmdSteps,nOpt,optWallMean,optWallMax,maxQueue,safetyPASS,effFlag';
if ~exist(outCsv, 'file')
    fid = fopen(outCsv, 'w'); fprintf(fid, '%s\n', hdr); fclose(fid);
end

for i = sel
    s = list{i};
    name = sprintf('M%02d_%s_%s_pen%.0f_seed%d', i, s.pattern.name, s.level.name, 100*s.pen, s.seed);
    flows = struct('route', s.pattern.routes, 'vph', num2cell(round(s.pattern.vph * s.level.scale)), 'pen', num2cell(s.pen * ones(1, numel(s.pattern.routes))));
    opts = struct('duration', 300, 'seed', s.seed, 'departSpeed', s.departSpeed);
    fprintf('\n[%s] %s\n', datestr(now, 'HH:MM:SS'), name);
    r = run_scenario(name, flows, opts);
    if ~r.ok
        row = sprintf('%d,%s,%s,%s,%.2f,%d,%s,,,,,,,,,,,,,,,,,,,,FAIL:%s', i, name, s.pattern.name, s.level.name, s.pen, s.seed, s.departSpeed, strrep(r.err, ',', ';'));
    else
        js = 0; if isfield(r.audit, 'jStops'), js = r.audit.jStops; end
        yc = 0; if isfield(r.audit, 'yellowCrossings'), yc = r.audit.yellowCrossings; end
        tripGap = r.tripCAV - r.tripHDV; if isnan(tripGap), tripGap = 0; end
        ci = 0; if isfield(r.audit, 'cutins'), ci = r.audit.cutins; end
        safety = r.collisions == 0 && r.violations == 0 && r.wasted == 0 && js == 0 && r.cannotStop == 0 && ...
               (isempty(r.minH) || r.minH >= -0.1) && abs(r.ratioCAV - 1) < 0.02 && ...
               (~isfield(r, 'optWallMax') || r.optWallMax < 1.0);
        effFlag = tripGap > 5;
        mh = NaN; if ~isempty(r.minH), mh = r.minH; end
        owm = NaN; owx = NaN; if isfield(r, 'optWallMean'), owm = r.optWallMean; owx = r.optWallMax; end
        row = sprintf('%d,%s,%s,%s,%.2f,%d,%s,%d,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%.2f,%d,%.3f,%d,%d,%d,%.2f,%.2f,%s,%d,%d', ...
            i, name, s.pattern.name, s.level.name, s.pen, s.seed, s.departSpeed, r.nCAV, r.nHDV, r.tripCAV, r.tripHDV, ...
            r.collisions, r.warnCAV, r.violations, yc, r.wasted, js, r.cannotStop, mh, ci, r.ratioCAV, r.uMinSteps, r.cmdSteps, ...
            r.nOpt, owm, owx, strrep(mat2str(r.maxQueue), ',', ' '), safety, effFlag);
    end
    fid = fopen(outCsv, 'a'); fprintf(fid, '%s\n', row); fclose(fid);
end
fprintf('\nMatrix finished. Results in %s\n', outCsv);
end
