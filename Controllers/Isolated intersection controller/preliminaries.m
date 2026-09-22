% -------------------------------------------------------------------------
% PRELIMINARIES: launch SUMO-GUI as a TraCI server and load traci4matlab.
% Isolated-intersection version. Uses the same launch options as the
% Sioux Falls (network) code, in particular --step-length 0.1.
% -------------------------------------------------------------------------

sumoExe   = fullfile(getenv("SUMO_HOME"), "bin", "sumo-gui.exe");
scriptDir = fileparts(mfilename('fullpath'));
cfg       = fullfile(scriptDir, "quickstart.sumocfg");
port      = 1234;   % any free port; change if another SUMO/TraCI pair is running

% Simulation step length [s]. Must equal dt in run_from_matlab.m and the
% <step-length> in quickstart.sumocfg.
stepLength = 0.1;

% Position-update scheme. 'ballistic' gives x(k+1) = x(k) + (v(k)+v(k+1))/2*dt,
% i.e. exactly the double-integrator update v*dt + 0.5*u*dt^2 of the paper's
% Eq. (10) when the controller commands v(k+1) = v(k) + u*dt.
% Set to false for SUMO's default Euler update x(k+1) = x(k) + v(k+1)*dt.
useBallistic = true;

% Collision handling. Default SUMO behaviour is to teleport the follower away
% ('teleport'), which hides safety failures. 'warn' lets the run continue and
% logs every collision to collisions.xml so report_collisions.m can show them.
collisionAction = 'warn';
collisionFile   = fullfile(scriptDir, 'collisions.xml');
if exist(collisionFile, 'file'), delete(collisionFile); end

extraOpts = sprintf('--collision.action %s --collision.check-junctions true --collision-output "%s" --log "%s"', ...
                    collisionAction, collisionFile, fullfile(scriptDir, 'sumo_log.txt'));
if useBallistic
    extraOpts = [extraOpts ' --step-method.ballistic true'];
end
if exist('SUMO_SEED', 'var')
    extraOpts = [extraOpts sprintf(' --seed %d', SUMO_SEED)];
end

% Launch SUMO-GUI detached (MATLAB will NOT block)
cmd = sprintf('start "" /B "%s" -c "%s" --remote-port %d --num-clients 1 --start --delay 1 --step-length %g %s', ...
              sumoExe, cfg, port, stepLength, extraOpts);
system(cmd);

% Wait briefly for the TraCI server to start listening
pause(1.0);

% -------------------------------------------------------------------------
% Load traci4matlab from its install location
% -------------------------------------------------------------------------
traciFolder = 'C:\Program Files\MATLAB\traci4matlab';

if ~exist(traciFolder,'dir')
    error('traci4matlab folder not found at: %s', traciFolder);
end

jarPath = fullfile(traciFolder,'traci4matlab.jar');
if exist(jarPath,'file')
    javaaddpath(jarPath);
else
    jars = dir(fullfile(traciFolder,'*.jar'));
    if ~isempty(jars)
        javaaddpath(fullfile(jars(1).folder, jars(1).name));
    else
        error('No traci4matlab jar file found in %s', traciFolder);
    end
end

addpath(traciFolder);
addpath(genpath(traciFolder));

% Clear any workspace variable named `traci` that could shadow the package
if exist('traci','var')
    clear traci
end

rehash;

% Verify traci.init is available
if isempty(which('traci.init'))
    listing = dir(traciFolder);
    names = {listing.name};
    error(['traci.init not found on MATLAB path. ',...
           'Expected +traci/init.m under traci4matlab. ',...
           'traci4matlab contents: %s'], strjoin(names, ', '));
end
