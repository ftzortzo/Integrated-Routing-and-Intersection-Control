function n = report_collisions(collisionFile)
% REPORT_COLLISIONS  Print the collisions SUMO logged (--collision-output).
%   With --collision.action warn, SUMO keeps simulating after a collision and
%   writes one <collision .../> element per event. Each is printed here.
%   Returns the number of collisions found (0 if the file is absent/empty).

n = 0;
if nargin < 1 || isempty(collisionFile)
    collisionFile = 'collisions.xml';
end
if ~exist(collisionFile, 'file')
    fprintf('\n===== Collisions: no collision file found (%s) =====\n\n', collisionFile);
    return;
end

txt = fileread(collisionFile);
tok = regexp(txt, '<collision\s+([^>]*)/?>', 'tokens');
n = numel(tok);

fprintf('\n===== Collisions logged by SUMO: %d =====\n', n);
if n == 0
    fprintf('(none)\n');
else
    fprintf('%-8s %-12s %-12s %-12s %-14s %-8s %s\n', 'time', 'type', 'lane', 'pos', 'collider', 'victim', 'collider_speed');
    for i = 1:min(n, 50)
        a = tok{i}{1};
        g = @(name) local_attr(a, name);
        fprintf('%-8s %-12s %-12s %-12s %-14s %-8s %s\n', g('time'), g('type'), g('lane'), g('pos'), g('collider'), g('victim'), g('colliderSpeed'));
    end
    if n > 50
        fprintf('... (%d more)\n', n - 50);
    end
end
fprintf('==========================================\n\n');
end

function v = local_attr(attrStr, name)
m = regexp(attrStr, [name '="([^"]*)"'], 'tokens', 'once');
if isempty(m)
    v = '-';
else
    v = m{1};
end
end
