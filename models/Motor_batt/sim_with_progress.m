function out = sim_with_progress(mdl, opts)
%SIM_WITH_PROGRESS  Run a Simulink model with a live progress bar (% / ETA).
%   out = sim_with_progress('VTOLTiltrotor')     % use in place of sim(mdl)
% Starts the sim async and polls SimulationTime; GUI waitbar (with Cancel) when a
% display exists, else prints to the command window. Returns the SimulationOutput.

if nargin < 2, opts = struct; end
if ~isfield(opts,'poll'),       opts.poll = 0.25;  end
if ~isfield(opts,'printEvery'), opts.printEvery = 5; end

load_system(mdl);

% resolve StopTime (may be a workspace expression)
sp = get_param(mdl,'StopTime');  Tstop = str2double(sp);
if isnan(Tstop), try, Tstop = evalin('base', sp); catch, Tstop = inf; end; end

useGui = usejava('desktop') && feature('ShowFigureWindows');
h = [];
if useGui
    try
        h = waitbar(0, 'starting...', 'Name', ['Sim: ' mdl], ...
            'CreateCancelBtn', 'setappdata(gcbf,''cancel'',1)');
        setappdata(h, 'cancel', 0);
    catch, useGui = false; end
end

set_param(mdl, 'SimulationCommand', 'start');   % non-blocking
t0 = tic; lastPrint = -inf; started = false;
while true
    st = get_param(mdl, 'SimulationStatus');
    if ~started
        if ~strcmp(st, 'stopped'), started = true;
        elseif toc(t0) > 15
            fprintf(2, '  [%s] simulation did not start\n', mdl); break;
        end
    elseif strcmp(st, 'stopped'), break;
    end
    tsim = get_param(mdl, 'SimulationTime');
    frac = max(0, min(tsim / Tstop, 1));   el = toc(t0);
    eta  = el / max(frac, 1e-6) * (1 - frac);
    msg  = sprintf('%.1f%%   t = %.4g / %.4g s   |   elapsed %s   ETA %s', ...
                   100*frac, tsim, Tstop, hms(el), hms(eta));
    if useGui && ishghandle(h)
        waitbar(frac, h, msg);
        if getappdata(h, 'cancel')
            set_param(mdl, 'SimulationCommand', 'stop');
            fprintf(2, '  [%s] cancelled at %.1f%%\n', mdl, 100*frac); break;
        end
    elseif (el - lastPrint) >= opts.printEvery
        fprintf('  [%s] %s\n', mdl, msg);  lastPrint = el;
    end
    pause(opts.poll);
end
if useGui && ishghandle(h), delete(h); end

tw = tic;
while ~strcmp(get_param(mdl,'SimulationStatus'),'stopped') && toc(tw) < 30, pause(0.1); end
fprintf('  [%s] finished in %s\n', mdl, hms(toc(t0)));

out = [];
try
    if strcmp(get_param(mdl,'ReturnWorkspaceOutputs'), 'on')
        nm = get_param(mdl, 'ReturnWorkspaceOutputsName');
        if evalin('base', sprintf('exist(''%s'',''var'')', nm)), out = evalin('base', nm); end
    end
catch
end
end

function s = hms(t)
if isinf(t) || isnan(t), s = '--'; return; end
t = round(t);
if t >= 3600, s = sprintf('%d:%02d:%02d', floor(t/3600), mod(floor(t/60),60), mod(t,60));
else,         s = sprintf('%d:%02d', floor(t/60), mod(t,60)); end
end
