function run_compare_all()
% Drive Simulation_Compare_run over all matched profiles.
% Solid = SA-88 (sa88_batch), Dash-dot = Prof. Jung (Prof_Jung_Battery).

% flightState enum must exist so Flight_Mode (saved as enum) loads/reads.
if exist('flightState','class') ~= 8
    Simulink.defineIntEnumType('flightState', ...
        {'Hover','Transition','FixedWing','BackTransition'}, [0;1;2;3], 'StorageType','uint8');
end

sa_root = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output\sa88_batch_20260613_075409';
pj_root = 'E:\file\chrome\Prof_Jung_Battery';
out_dir = 'E:\file\chrome\eVTOL_Simulator_260511_v15\output';

% matched (identical MTOW/Wind/Temp) profiles
pairs = {
    2 , 'P2_MTOW_5600_Wind_7_Temp_45';
    3 , 'P3_MTOW_7000_Wind_5_Temp_20';
    4 , 'P4_MTOW_7000_Wind_0_Temp_25';
    5 , 'P5_MTOW_7000_Wind_0_Temp_45';
    6 , 'P6_MTOW_5600_Wind_5_Temp_45';
    7 , 'P7_MTOW_7000_Wind_0_Temp_20';
    9 , 'P9_MTOW_6020_Wind_7_Temp_45';
    10, 'P10_MTOW_6020_Wind_7_Temp_20'};

for k = 1:size(pairs,1)
    prof = pairs{k,1}; name = pairs{k,2};
    f_sa = fullfile(sa_root, name, [name '.mat']);
    f_pj = fullfile(pj_root, [name '.mat']);
    out  = fullfile(out_dir, sprintf('Compare_%s.png', name));
    fprintf('=== P%d : %s ===\n', prof, name);
    if ~exist(f_sa,'file'); fprintf(2,'  missing SA-88: %s\n', f_sa); continue; end
    if ~exist(f_pj,'file'); fprintf(2,'  missing Jung : %s\n', f_pj); continue; end
    try
        Simulation_Compare_run(prof, f_sa, f_pj, out);
    catch ME
        fprintf(2,'  P%d FAILED: %s\n', prof, ME.message);
    end
end
fprintf('ALL DONE. Figures in %s\n', out_dir);
end
