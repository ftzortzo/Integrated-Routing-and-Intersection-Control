function R = audit_controller(simData)
% AUDIT_CONTROLLER  Behavioural X-ray of the CAV controller from simData.stepLog.
%
%   R = audit_controller(simData)
%
% Reports: every stop-line crossing and the TLS state at that instant
% (must be G), crossing speeds and headways; every full stop (where it came
% to rest, standstill gap, launch delay after green); leaderless
% decelerations on green and their cause; rear-end constraint (min h, min
% gap); downstream policy gaps; control effort; hard-braking episodes.

L = simData.stepLog;
t = L.t(:); vid = L.vid(:); lane = L.lane(:); v = L.speed(:); u = L.cmd_u(:); d = L.distToStop(:);
tt = L.t_target(:); un = L.u_nom(:); cbf = logical(L.cbf_active(:)); h = L.h(:); gap = L.gap(:);
ldr = L.leader(:); sa = logical(L.stopActive(:)); gn = logical(L.greenNow(:)); ge = L.greenEnd(:);
prof = L.profile(:); ss = logical(L.standingStart(:)); tls = L.tls(:);
cmd = ~isnan(u);
dt = simData.dt;
gm = 1.0; if isfield(simData, 'cav') && isfield(simData.cav, 'green_margin'), gm = simData.cav.green_margin; end
hasLeader = ~cellfun(@isempty, ldr);
vids = unique(vid);

R = struct();
fprintf('\n################ CONTROLLER AUDIT ################\n');
fprintf('CAVs: %d   commanded steps: %d\n', numel(vids), nnz(cmd));

% ---- crossings ----
linkmap = containers.Map({'NC_0','NC_1','EC_0','EC_1','SC_0','SC_1','WC_0','WC_1'}, ...
                         {[1 2],[3],[4 5],[6],[7 8],[9],[10 11],[12]});
cross = {};   % {t, vid, lane, speed, tlsChars}
for k = 1:numel(vids)
    idx = find(strcmp(vid, vids{k})); [~, o] = sort(t(idx)); idx = idx(o);
    for j = 2:numel(idx)
        a = idx(j-1); b = idx(j);
        if isKey(linkmap, lane{a}) && ~isempty(lane{b}) && lane{b}(1) == ':'
            chars = '';
            if ~isempty(tls{a}), chars = tls{a}(linkmap(lane{a})); end
            cross(end+1, :) = {t(b), vids{k}, lane{a}, v(b), chars}; %#ok<AGROW>
        end
    end
end
nC = size(cross, 1);
notGreen = false(nC, 1); onYellow = false(nC, 1);
for i = 1:nC
    onYellow(i) = ~any(ismember(cross{i,5}, 'Gg')) && any(cross{i,5} == 'y');
    notGreen(i) = ~any(ismember(cross{i,5}, 'Ggy'));
end
fprintf('\n== CROSSINGS: %d   on green: %d   on clearance (yellow, admissible if junction cleared): %d   ON RED: %d ==\n', nC, nC - nnz(onYellow) - nnz(notGreen), nnz(onYellow), nnz(notGreen));
for i = find(notGreen)'
    fprintf('   VIOLATION t=%.1f %s from %s at %.2f m/s, TLS=%s\n', cross{i,1}, cross{i,2}, cross{i,3}, cross{i,4}, cross{i,5});
end
if nC > 0
    sp = cell2mat(cross(:,4));
    fprintf('crossing speed: min %.2f  mean %.2f  max %.2f\n', min(sp), mean(sp), max(sp));
    lanesC = unique(cross(:,3));
    for i = 1:numel(lanesC)
        ts = sort(cell2mat(cross(strcmp(cross(:,3), lanesC{i}), 1)));
        hw = diff(ts);
        if ~isempty(hw), fprintf('   %s: %d crossings, headway min %.2f s median %.2f s\n', lanesC{i}, numel(ts), min(hw), median(hw)); end
    end
end
R.crossings = cross; R.violations = nnz(notGreen); R.yellowCrossings = nnz(onYellow);

% ---- stops ----
fprintf('\n== STOPS (v < 0.05 m/s while commanded on an incoming lane) ==\n');
nStops = 0;
for k = 1:numel(vids)
    idx = find(strcmp(vid, vids{k}) & cmd); [~, o] = sort(t(idx)); idx = idx(o);
    st = v(idx) < 0.05; j = 1;
    while j <= numel(idx)
        if st(j)
            j0 = j; while j <= numel(idx) && st(j), j = j + 1; end
            i0 = idx(j0); i1 = idx(j-1);
            tl = NaN; for a = idx(j:end)', if v(a) > 0.5, tl = t(a); break; end, end
            gs = NaN; for a = idx(j0:end)', if gn(a) || (ss(a)), gs = t(a); break; end, end
            nStops = nStops + 1;
            if hasLeader(i0), gapStr = sprintf('gap to %s = %.2f m', ldr{i0}, gap(i0)); else, gapStr = 'no leader'; end
            fprintf('   %s stopped at t=%.1f for %.1f s, %.2f m before the line (%s), launched %.1f s after green\n', ...
                vids{k}, t(i0), t(i1) - t(i0), d(i0), gapStr, tl - gs);
        else
            j = j + 1;
        end
    end
end
if nStops == 0, fprintf('   (none)\n'); end
R.nStops = nStops;

% ---- wasted greens: a CAV stopped at the line at green start, no CAV crossing from that lane during the green ----
if isfield(simData, 'qLog') && ~isempty(simData.qLog.t)
    Qt = simData.qLog.t(:); Qtls = simData.qLog.tls(:);
    lanePhase = containers.Map({'NC_0','NC_1','EC_0','EC_1','SC_0','SC_1','WC_0','WC_1'}, {1,2,3,4,1,2,3,4});
    phaseOf = @(s) (~isempty(s)) * ( (numel(s)>=3 && s(3)=='G')*2 + (numel(s)>=1 && s(1)=='G')*1 + (numel(s)>=5 && (s(4)=='G'||s(5)=='G'))*3 + (numel(s)>=6 && s(6)=='G')*4 );
    ph = zeros(numel(Qt),1); for i = 1:numel(Qt), ph(i) = phaseOf(Qtls{i}); end
    fprintf('\n== WASTED GREENS (CAV stopped at the line at green start, no CAV crossing from its lane) ==\n');
    nW = 0; i = 1;
    while i <= numel(Qt)
        if ph(i) > 0
            j = i; while j <= numel(Qt) && ph(j) == ph(i), j = j + 1; end
            tg0 = Qt(i); tg1 = Qt(j-1) + 1; p = ph(i);
            if tg1 >= Qt(end) - 0.5, i = j; continue; end   % window cut off by the end of the run
            % CAVs stopped within 3 m of the line on a lane of phase p at tg0 (+-0.6 s)
            m = cmd & abs(t - tg0) <= 0.6 & v < 0.05 & d < 3;
            lanesAtLine = unique(lane(m));
            for li = 1:numel(lanesAtLine)
                ln = lanesAtLine{li};
                if ~isKey(lanePhase, ln) || lanePhase(ln) ~= p, continue; end
                crossed = false;
                for c = 1:nC
                    if strcmp(cross{c,3}, ln) && cross{c,1} >= tg0 && cross{c,1} <= tg1 + 3, crossed = true; break; end
                end
                if ~crossed
                    nW = nW + 1;
                    fprintf('   phase %d green %.0f-%.0f s (%.0f s): CAV stopped at line on %s did not cross\n', p, tg0, tg1, tg1 - tg0, ln);
                end
            end
            i = j;
        else
            i = i + 1;
        end
    end
    if nW == 0, fprintf('   (none)\n'); end
    R.wastedGreens = nW;
end

% ---- mid-junction stops ----
R.jStops = 0;
if isfield(simData, 'jStop')
    J = simData.jStop;
    fprintf('\n== MID-JUNCTION STOPS (vehicle at rest on an internal lane) ==\n');
    if isempty(J)
        fprintf('   (none)\n');
    else
        % group samples of the same vehicle into events (sort by vehicle, then time)
        [~, o] = sortrows([{J.vid}', num2cell([J.t]')], [1 2]); J = J(o);
        ev = {}; lastV = ''; lastT = -Inf;
        for i = 1:numel(J)
            if strcmp(J(i).vid, lastV) && J(i).t - lastT <= 1.0
                ev{end}.tEnd = J(i).t; %#ok<AGROW>
            else
                ev{end+1} = struct('vid', J(i).vid, 'type', J(i).type, 'lane', J(i).lane, 'tStart', J(i).t, 'tEnd', J(i).t, 'tls', J(i).tls, 'others', J(i).others); %#ok<AGROW>
            end
            lastV = J(i).vid; lastT = J(i).t;
        end
        R.jStops = numel(ev);
        for i = 1:min(numel(ev), 30)
            e = ev{i};
            fprintf('   %s (%s) stopped on %s from t=%.1f to %.1f (%.1f s), TLS=%s, others inside: %s\n', e.vid, e.type, e.lane, e.tStart, e.tEnd, e.tEnd - e.tStart, e.tls, e.others);
        end
        if numel(ev) > 30, fprintf('   ... (%d more)\n', numel(ev) - 30); end
    end
end

% ---- leaderless deceleration on green ----
sel = cmd & ~hasLeader & gn & (u < -0.3);
tot = nnz(cmd & ~hasLeader & gn);
nStop = nnz(sel & sa);
nLater = nnz(sel & ~sa & (tt > ge - gm + 1e-6));
nWithin = nnz(sel) - nStop - nLater;
fprintf('\n== LEADERLESS DECELERATION WHILE GREEN: %d of %d steps ==\n', nnz(sel), tot);
fprintf('   stop constraint active: %d | profile aims at a later green: %d | profile decelerates within green: %d\n', nStop, nLater, nWithin);

% ---- rear-end constraint ----
wl = cmd & hasLeader;
% cut-ins: leader changed to a different vehicle at a gap below s0 + v*T + 5 m;
% steps within 1 s after a cut-in are reported separately (not the barrier's doing)
cutin = false(size(t)); afterCut = false(size(t));
for k = 1:numel(vids)
    idx = find(strcmp(vid, vids{k}) & cmd); [~, o] = sort(t(idx)); idx = idx(o);
    for j = 2:numel(idx)
        a = idx(j-1); b = idx(j);
        if ~isempty(ldr{b}) && ~strcmp(ldr{a}, ldr{b}) && h(b) < 5
            cutin(b) = true;
            afterCut(idx(t(idx) >= t(b) & t(idx) <= t(b) + 1.0)) = true;
        end
    end
end
fprintf('\n== REAR-END CONSTRAINT (Eq. 18) ==\n');
if any(wl)
    wlc = wl & ~afterCut;
    fprintf('   steps with leader %d, CBF active %d, min h = %.2f m excluding cut-ins (steps with h<0: %d), min bumper gap = %.2f m\n', ...
        nnz(wl), nnz(cbf & wl), min(h(wlc)), nnz(h(wlc) < 0), min(gap(wlc)));
    fprintf('   cut-ins (leader changed at h < 5 m): %d, min h during the second after a cut-in: %.2f m\n', nnz(cutin), min([h(wl & afterCut); Inf]));
end
R.min_h = min(h(wl & ~afterCut)); R.min_gap = min(gap(wl & ~afterCut)); R.cutins = nnz(cutin); R.min_h_cutin = min([h(wl & afterCut); Inf]);

% ---- downstream ----
if isfield(simData, 'dsLog') && ~isempty(simData.dsLog.t)
    D = simData.dsLog;
    fprintf('\n== DOWNSTREAM POLICY (after the stop line) ==\n');
    fprintf('   commands %d, min gap %.2f m, u=u_max %.0f%%, u<0 %.0f%%, min speed %.2f m/s\n', ...
        numel(D.t), min(D.gap), 100*mean(D.u >= 3.458), 100*mean(D.u < 0), min(D.v));
end

% ---- effort & profiles ----
e = [];
for k = 1:numel(vids)
    m = strcmp(vid, vids{k}) & cmd;
    if nnz(m) > 10, e(end+1) = mean(u(m).^2); end %#ok<AGROW>
end
fprintf('\n== CONTROL EFFORT (mean u^2 per CAV in zone): mean %.3f  median %.3f  max %.3f ==\n', mean(e), median(e), max(e));
[pu, ~, ic] = unique(prof(cmd)); cnt = accumarray(ic, 1);
fprintf('   profile cases: ');
for i = 1:numel(pu), fprintf('%s=%d  ', pu{i}, cnt(i)); end
fprintf('\n   u = u_min steps: %d   u = u_max steps: %d   stop-constraint active: %d   standing starts: %d\n', ...
    nnz(u <= -2.999), nnz(u >= 3.458), nnz(sa), nnz(ss));

% ---- hard braking episodes ----
fprintf('\n== HARD BRAKING (u <= -2.5 for >= 1 s) ==\n');
nE = 0;
for k = 1:numel(vids)
    idx = find(strcmp(vid, vids{k}) & cmd); [~, o] = sort(t(idx)); idx = idx(o);
    hb = u(idx) <= -2.5; j = 1;
    while j <= numel(idx)
        if hb(j)
            j0 = j; while j <= numel(idx) && hb(j), j = j + 1; end
            if (j - j0) * dt >= 1
                nE = nE + 1;
                fprintf('   %s at t=%.1f for %.1f s from v=%.1f m/s, d=%.1f m (target %.1f s, %s)\n', ...
                    vids{k}, t(idx(j0)), (j - j0) * dt, v(idx(j0)), d(idx(j0)), tt(idx(j0)), prof{idx(j0)});
            end
        else
            j = j + 1;
        end
    end
end
if nE == 0, fprintf('   (none)\n'); end
fprintf('##################################################\n\n');
end
