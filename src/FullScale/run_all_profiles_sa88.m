function run_all_profiles_sa88(profiles_to_run)
%RUN_ALL_PROFILES_SA88 Batch-run the eVTOL simulator over multiple flight
% profiles using the SA-88 1-RC battery model. For EACH profile it creates
% its own output folder and saves there:
%   - outTuned (+ companion setup structs) as a .mat
%   - every generated figure as a .fig and a .png

%
% Usage (run from the PROJECT ROOT, i.e. the kiat_battery_project folder):
%   run_all_profiles_sa88              % runs profiles 1..11
%   run_all_profiles_sa88(1:11)        % same as above
%   run_all_profiles_sa88([2 5 8])     % run only the listed profiles

if nargin < 1 || isempty(profiles_to_run)
    profiles_to_run = 1:11;
end

%% ===== One-time setup =====
addpath(genpath('src'));

bdclose('all');
close all;
load_system('VTOLTiltrotor');
load_system('VTOLDynamics');

if exist('flightState', 'class') ~= 8
    Simulink.defineIntEnumType('flightState', ...
        {'Hover','Transition','FixedWing','BackTransition'}, [0;1;2;3], ...
        'StorageType', 'uint8');
end

batch_ts = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
out_root = fullfile('output', sprintf('sa88_batch_%s', batch_ts));
if ~exist(out_root, 'dir')
    mkdir(out_root);
end

log_path = fullfile(out_root, 'batch_log.txt');
log_fid  = fopen(log_path, 'w');
fprintf(log_fid, 'SA-88 batch run start: %s\n', batch_ts);
fprintf(log_fid, 'Profiles: %s\n\n', mat2str(profiles_to_run));

fprintf('\n========================================\n');
fprintf(' SA-88 batch run\n');
fprintf(' Profiles : %s\n', mat2str(profiles_to_run));
fprintf(' Output   : %s\n', out_root);
fprintf('========================================\n');

batch_t0 = tic;

%% ===== Loop profiles =====
for kk = 1:length(profiles_to_run)
    Profile = profiles_to_run(kk);
    fprintf('\n--- Profile %d  (%d/%d) ---\n', Profile, kk, length(profiles_to_run));
    p_t0 = tic;

    try
        % ===== Fresh environment for THIS profile (mirrors startup.m) =====
        cleanup;

        mdl = 'VTOLTiltrotor';
        load_system(mdl);

        motorctrl.p = 0.4; motorctrl.i = 0.001; motorctrl.d = 0; motorctrl.n = 100;
        Limit = 700; 

        cfgRefTop = getActiveConfigSet('VTOLTiltrotor');
        cfgTop    = getRefConfigSet(cfgRefTop);
        set_param(cfgTop, 'SolverType', 'Fixed-step', 'Solver', 'ode4');
        Ts        = 0.001; 
        mdlSub    = 'VTOLDynamics';
        load_system(mdlSub);
        cfgSubActive = getActiveConfigSet(mdlSub); 

        xGround=0; yGround=0; zGround=0;          
        iniRoll=0; iniYaw=0/180*pi; initPitch=0;  
        iniP=0; iniQ=0; iniR=0;                   

        load('data\contact.mat');                
        load_ctrl_interface();
        load_digital_twin_interface;
        load_controller_gains;                     % fresh VTOLcontrolGains
        const = load_const();

        % --- MTOW per profile ---
        switch Profile
            case {1,2,6},     eVTOL_MTOW = 5600;
            case {9,10},      eVTOL_MTOW = 6020;
            case {3,4,5,7,8}, eVTOL_MTOW = 7000;
            case 11,          eVTOL_MTOW = 5600;
            otherwise, error('Unknown Profile %d', Profile);
        end

        % --- Temperature per profile ---
        switch Profile
            case {1,3,7,10},  target_temperature = 20;
            case {4,11},      target_temperature = 25;
            case {2,5,6,8,9}, target_temperature = 45;
        end

        % --- Wind per profile ---
        switch Profile
            case {1,4,5,7},  Wind = 0; Wind_Speed = 0;
            case {3,6},      Wind = 1; Wind_Speed = 5; 
            case {2,8,9,10}, Wind = 1; Wind_Speed = 7; 
            case 11,         Wind = 1; Wind_Speed = 5;
        end

        % --- Yaw wind moment (mirrors startup.m) ---
        if Wind_Speed == 0
            Yaw_Wind_Moment = 0; 
        elseif (Wind_Speed == 5) && (eVTOL_MTOW == 5600), Yaw_Wind_Moment = 1400;
        elseif (Wind_Speed == 5) && (eVTOL_MTOW == 6020), Yaw_Wind_Moment = 2500;
        elseif (Wind_Speed == 5) && (eVTOL_MTOW == 7000), Yaw_Wind_Moment = 3500;
        elseif (Wind_Speed == 7) && (eVTOL_MTOW == 5600), Yaw_Wind_Moment = 3000;
        elseif (Wind_Speed == 7) && (eVTOL_MTOW == 6020), Yaw_Wind_Moment = 3000;
        elseif (Wind_Speed == 7) && (eVTOL_MTOW == 7000), Yaw_Wind_Moment = 3000;
        end

        [uavParams, HEV_Param] = load_vtol_dynamics_7000lb( ...
            const, Profile, eVTOL_MTOW, target_temperature);

        % Profile 8 vertical-axis gain bump (matches startup.m)
        if Profile == 8
            VTOLcontrolGains.P_Z  = 1.5 * VTOLcontrolGains.P_Z;
            VTOLcontrolGains.P_VZ = 1.5 * VTOLcontrolGains.P_VZ;
            VTOLcontrolGains.I_VZ = 1.5 * VTOLcontrolGains.I_VZ;
            VTOLcontrolGains.D_VZ = 1.5 * VTOLcontrolGains.D_VZ;
        end

        controlParams       = load_controller_parameters(uavParams, const);
        Visualization       = 0;    
        SensorType          = 0;     
        TuningMode          = 0;     
        Deployment          = false; 
        vIni                = 0 * const.kts2mps; 

        setupHoverConfiguration_mod;
        setupTransitionGuidanceMission_mod;

        configObj = getActiveConfigSet('VTOLAutopilotController');
        set_param(configObj, 'SourceName', 'VTOLConfiguration');
        transition_throttle = 0.2; 

        pack_kWh = (HEV_Param.Battery_Pack_Capacity * HEV_Param.Battery_Pack_Voltage)/1000;
        fprintf('  MTOW=%d lb, T=%d C, Wind=%d m/s, Pack=%.2f kWh @ %.0f V\n', ...
            eVTOL_MTOW, target_temperature, Wind_Speed, pack_kWh, ...
            HEV_Param.Battery_Pack_Voltage);

        % --- This profile's output folder ---
        prof_tag = sprintf('P%d_MTOW_%d_Wind_%d_Temp_%d', ...
                           Profile, eVTOL_MTOW, Wind_Speed, target_temperature);
        prof_dir = fullfile(out_root, prof_tag);
        if ~exist(prof_dir, 'dir')
            mkdir(prof_dir);
        end

        % --- Push function-local vars to base workspace so Simulink can
        %     resolve parameter names (sim() defaults to base workspace). ---
        denylist = {'log_fid','out_root','batch_ts','batch_t0','p_t0', ...
                    'profiles_to_run','kk','log_path','ME','mat_name', ...
                    'figs','j','base','fig_err','sim_dur','denylist','v', ...
                    'vars','prof_dir','prof_tag','err_path','mat_info', ...
                    'mat_MB','pack_kWh','sim_data','plot_ok','plot_msg', ...
                    'plot_err','status_tag'};
        vars = who;
        for v = 1:numel(vars)
            if ~ismember(vars{v}, denylist) && isvarname(vars{v})
                assignin('base', vars{v}, eval(vars{v}));
            end
        end

        % --- Simulate ---
        outTuned = sim(mdl);
        sim_dur  = toc(p_t0);

        sim_figs = findall(groot, 'Type', 'figure');

        % --- Save outTuned .mat into the profile folder ---
        mat_name = fullfile(prof_dir, [prof_tag '.mat']);
        save(mat_name, 'outTuned', 'TransitionMission', 'HEV_Param', ...
             'controlParams', 'uavParams', 'const', '-v7.3');
        mat_info = dir(mat_name);
        mat_MB   = mat_info.bytes / 1e6;

        plot_ok  = true;
        plot_msg = '';
        try
            if Profile == 11
                Simulation_Plot_Debugging();
            else
                Simulation_Plot();
            end
        catch plot_err
            plot_ok  = false;
            plot_msg = plot_err.message;
            fprintf(2, '  P%d plot failed (data already saved): %s\n', Profile, plot_msg);
            fprintf(log_fid, 'P%-2d  plot FAIL: %s\n', Profile, plot_msg);
        end

        % --- Save ONLY the analysis figures 
        figs = setdiff(findall(groot, 'Type', 'figure'), sim_figs);
        figs = flipud(figs(:));   % oldest first
        for j = 1:numel(figs)
            base = fullfile(prof_dir, sprintf('P%d_fig%d', Profile, j));
            try
                savefig(figs(j), [base '.fig']);
                saveas(figs(j),  [base '.png']);
            catch fig_err
                fprintf(2, '  Figure %d save failed: %s\n', j, fig_err.message);
            end
        end
        if ~isempty(figs)
            close(figs);          % close only the analysis figures
        end

        if plot_ok
            status_tag = 'OK';
        else
            status_tag = 'OK(data); PLOT-FAILED';
        end
        fprintf('  %s in %.1fs  (mat=%.1f MB, %d figs)  -> %s\n', ...
            status_tag, sim_dur, mat_MB, numel(figs), prof_dir);
        fprintf(log_fid, 'P%-2d  %-22s sim=%6.1fs  mat=%6.1f MB  figs=%d  %s\n', ...
            Profile, status_tag, sim_dur, mat_MB, numel(figs), prof_tag);

    catch ME
        fprintf(2, '  P%d FAILED\n', Profile);
        fprintf(log_fid, 'P%-2d  FAIL\n', Profile);
        dump_exception(log_fid, ME, 0);
        % Save the MException for post-mortem 
        try
            if exist('prof_dir', 'var') && exist(prof_dir, 'dir')
                err_path = fullfile(prof_dir, sprintf('P%d_error.mat', Profile));
            else
                err_path = fullfile(out_root, sprintf('P%d_error.mat', Profile));
            end
            save(err_path, 'ME', '-v7.3');
            fprintf('  Error details saved to %s\n', err_path);
        catch
        end
        % Close any analysis figures from this failed profile, but NEVER the
        % animation figure 
        try
            if exist('sim_figs', 'var')
                leftover = setdiff(findall(groot, 'Type', 'figure'), sim_figs);
                if ~isempty(leftover), close(leftover); end
            end
        catch
        end
    end
end

batch_dur_min = toc(batch_t0) / 60;
fprintf(log_fid, '\nTotal batch time: %.1f min\n', batch_dur_min);
fclose(log_fid);

fprintf('\n========================================\n');
fprintf(' All profiles done. Total: %.1f min\n', batch_dur_min);
fprintf(' Output: %s\n', out_root);
fprintf(' Log:    %s\n', log_path);
fprintf('========================================\n');

end


function dump_exception(fid, ME, depth)
% Recursively write an MException and its causes to a file handle and the
% command window, indented by depth. 
indent = repmat('  ', 1, depth + 1);
fprintf(fid, '%s[%s] %s\n', indent, ME.identifier, ME.message);
fprintf(2,   '%s[%s] %s\n', indent, ME.identifier, ME.message);
for s = 1:length(ME.stack)
    fprintf(fid, '%s  at %s (line %d)\n', indent, ME.stack(s).name, ME.stack(s).line);
end
for c = 1:length(ME.cause)
    fprintf(fid, '%s---- cause %d ----\n', indent, c);
    fprintf(2,   '%s---- cause %d ----\n', indent, c);
    dump_exception(fid, ME.cause{c}, depth + 1);
end
end
