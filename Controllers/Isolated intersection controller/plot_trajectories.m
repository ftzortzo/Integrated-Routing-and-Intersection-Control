function plot_trajectories(simData, tRange)
% PLOT_TRAJECTORIES  Time-space diagram for each path with signal phases
%   and optimization events, using actual recorded TLS states from SUMO.
%
%   plot_trajectories(simData)
%   plot_trajectories(simData, [tStart tEnd])
%
%   Usage after simulation:
%     load('sim_trajectory_data.mat');
%     plot_trajectories(simData, [0 120]);

if nargin < 2 || isempty(tRange)
    tRange = [0, 120];
end

% Font size for all text (labels, title, ticks, legend)
fontSz = 26;

% Line width for trajectory curves
lineW = 2.5;
% Max approach distance per path (set to Inf to auto-detect)
% Index matches pathDefs order: 1 = N->E, 2 = E->W
maxApproachOverride = [245, 245];
trajLog   = simData.trajLog;
tlsLog    = simData.tlsLog;      % Nx1 time samples
tlsStates = simData.tlsStates;   % Nx1 cell of TLS state strings
optLog    = simData.optLog;
pathDefs  = simData.pathDefs;

if isempty(trajLog)
    warning('No trajectory data recorded.');
    return;
end

% Extract arrays for fast indexing
allT = [trajLog.t];
allDist = [trajLog.dist];
allPathIdx = [trajLog.pathIdx];
allVid = {trajLog.vid};
allType = {trajLog.typeID};

% Colors
cavColor = [0 0 1];        % blue
hdvColor = [1 0 1];        % magenta
greenColor = [0 0.7 0];
redColor   = [0.85 0 0];
yellowColor = [0.95 0.85 0];

% Figure position: keep toolbar visible by starting near top-left
scrSz = get(0, 'ScreenSize');
figH = min(500*numel(pathDefs), scrSz(4) - 100);
figTop = scrSz(4) - figH - 40;  % 40px from top for toolbar
figure('Position', [100 figTop 1400 figH], 'Color', 'w');

for pi = 1:numel(pathDefs)
    subplot(numel(pathDefs), 1, pi);
    hold on; box on;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

    pathMask = (allPathIdx == pi) & (allT >= tRange(1)) & (allT <= tRange(2));
    pathVids = unique(allVid(pathMask));

    % Determine max approach distance for this path
    negDists = allDist(pathMask & (allDist < 0));
    maxApproach = max(-negDists, [], 'omitnan');
    if isempty(maxApproach), maxApproach = 300; end
    if pi <= numel(maxApproachOverride) && isfinite(maxApproachOverride(pi))
        maxApproach = maxApproachOverride(pi);
    end
    postCrossingLimit = 0.5 * maxApproach;

    % Set axis limits early so signal bar is sized correctly
    yBot = -maxApproach;
    yTop = postCrossingLimit;
    ylim([yBot, yTop]);
    xlim(tRange);

    % --- Control zone entrance line at 200m ---
    yline(-200, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8, 'HandleVisibility', 'off');

    % =====================================================================
    % 1) Draw signal bar FIRST (behind everything)
    % =====================================================================
    barHeight = 0.03 * (yTop - yBot);
    barBottom = -barHeight / 2;  % centered on stop line (y=0)
    linkIdx = pathDefs(pi).tlsLinkIndices;

    for si = 1:numel(tlsLog)-1
        t1 = tlsLog(si);
        t2 = tlsLog(si+1);
        if t2 < tRange(1) || t1 > tRange(2), continue; end
        t1 = max(t1, tRange(1));
        t2 = min(t2, tRange(2));
        if t2 <= t1, continue; end

        stateStr = tlsStates{si};
        chars = stateStr(linkIdx);

        if any(chars == 'G' | chars == 'g')
            bColor = greenColor;
        elseif any(chars == 'y')
            bColor = yellowColor;
        else
            bColor = redColor;
        end

        fill([t1 t2 t2 t1], ...
             [barBottom barBottom barBottom+barHeight barBottom+barHeight], ...
             bColor, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end

    % =====================================================================
    % 2) Stop line removed per user request
    % =====================================================================

    % =====================================================================
    % 3) Optimization events (removed)
    % =====================================================================

    % =====================================================================
    % 4) Draw vehicle trajectories ON TOP
    % =====================================================================
    legendedCAV = false;
    legendedHDV = false;
    hCAV = []; hHDV = [];

    for vi = 1:numel(pathVids)
        vid = pathVids{vi};
        vehMask = pathMask & strcmp(allVid, vid);
        tVeh = allT(vehMask);
        dVeh = allDist(vehMask);

        % Clip post-crossing
        keepMask = dVeh <= postCrossingLimit;
        tVeh = tVeh(keepMask);
        dVeh = dVeh(keepMask);

        typeEntries = allType(vehMask);
        isCAV = strcmpi(typeEntries{1}, 'CAV');

        if isCAV
            color = cavColor;
            if ~legendedCAV
                hCAV = plot(tVeh, dVeh, '-', 'Color', color, 'LineWidth', lineW, ...
                    'DisplayName', 'CAV');
                legendedCAV = true;
            else
                plot(tVeh, dVeh, '-', 'Color', color, 'LineWidth', lineW, ...
                    'HandleVisibility', 'off');
            end
        else
            color = hdvColor;
            if ~legendedHDV
                hHDV = plot(tVeh, dVeh, '-', 'Color', color, 'LineWidth', lineW, ...
                    'DisplayName', 'HDV');
                legendedHDV = true;
            else
                plot(tVeh, dVeh, '-', 'Color', color, 'LineWidth', lineW, ...
                    'HandleVisibility', 'off');
            end
        end
    end

    % --- Format axes ---
    xlabel('Time (s)', 'FontSize', fontSz, 'Color', 'k');
    ylabel('Distance to stop line (m)', 'FontSize', fontSz, 'Color', 'k');
    title(sprintf('Path: %s', pathDefs(pi).name), 'FontSize', fontSz+2, 'Color', 'k');

    % Custom tick labels: fix the ticks explicitly (otherwise MATLAB re-ticks
    % on resize and the labels below go stale), then show abs() so both
    % sides read as positive distance. Stop line is at 0.
    ax = gca;
    ticks = (ceil(-maxApproach/50)*50):50:(floor(yTop/50)*50);
    ax.YTick = unique([ticks, 0]);
    ax.YTickLabel = arrayfun(@(v) num2str(abs(v)), ax.YTick, 'UniformOutput', false);

    set(gca, 'FontSize', fontSz, 'Layer', 'top', 'XGrid', 'off', 'YGrid', 'on', ...
        'GridColor', [0.5 0.5 0.5], 'GridAlpha', 0.15);

    % Legend with white background
    legHandles = [];
    legEntries = {};
    if ~isempty(hCAV)
        legHandles(end+1) = hCAV;
        legEntries{end+1} = 'CAV';
    end
    if ~isempty(hHDV)
        legHandles(end+1) = hHDV;
        legEntries{end+1} = 'HDV';
    end
    if ~isempty(legHandles)
        lg = legend(legHandles, legEntries, 'Location', 'northwest', 'FontSize', fontSz);
        lg.Color = 'w';
        lg.TextColor = 'k';
        lg.EdgeColor = [0.5 0.5 0.5];
    end

    hold off;
end

end
