%% Simulation_Plot_PMSM.m
% Battery and drive plots for the PMSM build.
%
% Simulation_Plot reads outTuned.Battery_Data.Batt, the bus of the system-level
% battery inside Uav_Electrical_lib. PMSM_Drive replaced that subsystem, so the
% bus is gone and the same call fails with
%     Unrecognized method, property, or field 'Batt' for class 'timeseries'
% The battery now comes from PMSM_Drive/Battery through the To Workspace blocks
% pmsm_Vdc, pmsm_SOC, pmsm_Idc, pmsm_Idq and pmsm_Iabc.
%
% Needs outTuned and uavParams in the workspace. Falls back to the old bus when
% it is present, so it also runs on a system-level result.

if ~exist('outTuned','var'), error('outTuned not found. load the run first.'); end
has = @(n) ismember(n, outTuned.who);

%% ---- battery signals, new build first then the old bus ----
if has('pmsm_Vdc')
    src   = 'PMSM_Drive/Battery';
    tB    = outTuned.pmsm_Vdc.Time(:);
    Vpack = outTuned.pmsm_Vdc.Data(:);
    SOC   = outTuned.pmsm_SOC.Data(:)*100;                 % [-] -> [%]
    Idc   = outTuned.pmsm_Idc.Data;
    if size(Idc,1) < size(Idc,2), Idc = Idc.'; end          % want N x 4
    Ipack = sum(Idc,2);
elseif has('Battery_Data') && isstruct(outTuned.Battery_Data)
    src   = 'Uav_Electrical_lib/Batt';
    b     = outTuned.Battery_Data.Batt;
    tB    = b.SOC____.Time(:);
    Vpack = b.Voltage__V_.Data(:);
    SOC   = b.SOC____.Data(:);
    Ipack = b.Current__A_.Data(:);
else
    fprintf(2,'outTuned holds: %s\n', strjoin(outTuned.who','; '));
    error(['No battery signals. Expected pmsm_Vdc/pmsm_SOC/pmsm_Idc from the ' ...
           'PMSM build, or Battery_Data.Batt from the system-level one.']);
end

Cap_Ah = 220.5;
if exist('uavParams','var') && isfield(uavParams,'Battery_Pack_Capacity')
    Cap_Ah = uavParams.Battery_Pack_Capacity;
end
Crate  = Ipack/Cap_Ah;
Pack_kW = Vpack.*Ipack/1000;
Energy_kWh = cumtrapz(tB, Pack_kW)/3600;

fprintf('battery from %s | %.0f Ah pack\n', src, Cap_Ah);
fprintf('  SOC   %.2f -> %.2f %%\n', SOC(1), SOC(end));
fprintf('  Vpack %.0f max, %.0f min V\n', max(Vpack), min(Vpack(Vpack>1)));
fprintf('  Ipack %.0f A peak, C-rate %.2f peak\n', max(Ipack), max(Crate));
fprintf('  energy %.2f kWh over %.0f s\n', Energy_kWh(end), tB(end));

%% ---- flight-mode intervals for the shading ----
Tv = [tB(end) tB(end) tB(end) tB(end)];
if has('Flight_Mode')
    fm = double(outTuned.Flight_Mode.Data); tF = outTuned.Flight_Mode.Time;
    ch = find(fm(1:end-1) ~= fm(2:end));
    for k = 1:min(numel(ch),4), Tv(k) = tF(ch(k)); end
end
MI = [0 Tv(1); Tv(1) Tv(2); Tv(2) Tv(3); Tv(3) Tv(4); Tv(4) 1e5];
MC = [0.15 0.15 0.15; 0.5 0.5 0.5; 0.9 0.9 0.9; 0.5 0.5 0.5; 0.15 0.15 0.15];
shade = @(ax) arrayfun(@(k) shade_one(ax, MI(k,:), MC(k,:)), 1:5);

%% ---- figure 1, the six battery panels ----
figure('Units','normalized','OuterPosition',[0.05 0.1 0.55 0.8],'Color','w');
pan = {SOC,'SOC','(%)'; Crate,'C-rate','(-)'; Ipack,'Pack current','(A)'; ...
       Vpack,'Pack voltage','(V)'; Pack_kW,'Required power','(kW)'; ...
       Energy_kWh,'Cumulative energy','(kWh)'};
for k = 1:6
    ax = subplot(2,3,k); plot(tB, pan{k,1}, 'LineWidth', 2); grid on
    title(pan{k,2},'FontSize',13,'FontWeight','bold');
    xlabel('Time (s)'); ylabel(pan{k,3}); xlim([0 tB(end)]);
    hold on; shade(ax); set(ax,'Layer','top');
end
sgtitle('Battery Pack Electrical Performance (PMSM build)','FontSize',17,'FontWeight','bold');

%% ---- figure 2, the drive currents ----
if has('pmsm_Iabc')
    tA = outTuned.pmsm_Iabc.Time(:); Ia = outTuned.pmsm_Iabc.Data;
    if size(Ia,1) < size(Ia,2), Ia = Ia.'; end
    fs = 1/mean(diff(tA(1:min(200,end))));
    fprintf('  Iabc  %d samples x %d ch at %.0f Hz, peak %.0f A\n', size(Ia,1), size(Ia,2), fs, max(abs(Ia(:))));

    figure('Units','normalized','OuterPosition',[0.05 0.1 0.55 0.7],'Color','w');
    subplot(2,1,1)
    plot(tA, Ia(:,1:3), 'LineWidth', 1); grid on
    title(sprintf('Motor 1 phase currents, whole mission (%.0f Hz)', fs),'FontSize',13,'FontWeight','bold');
    xlabel('Time (s)'); ylabel('(A)'); legend('i_a','i_b','i_c','Location','best');
    xlim([0 tA(end)]);

    % one 50 ms window in the takeoff hover, where the waveform is readable
    w = tA > 8 & tA < 8.05;
    if nnz(w) < 10, w = tA < 0.05; end
    subplot(2,1,2)
    plot(tA(w), Ia(w,1:3), 'LineWidth', 1.6); grid on
    title('Motor 1 phase currents, 50 ms window','FontSize',13,'FontWeight','bold');
    xlabel('Time (s)'); ylabel('(A)'); legend('i_a','i_b','i_c','Location','best');
end

if has('pmsm_Idq')
    tD = outTuned.pmsm_Idq.Time(:); Id = outTuned.pmsm_Idq.Data;
    if size(Id,1) < size(Id,2), Id = Id.'; end
    figure('Units','normalized','OuterPosition',[0.05 0.1 0.55 0.5],'Color','w');
    ax = axes; plot(tD, Id(:,1), tD, Id(:,2), 'LineWidth', 1.6); grid on; hold on
    shade(ax); set(ax,'Layer','top');
    title('Motor 1 dq currents','FontSize',13,'FontWeight','bold');
    xlabel('Time (s)'); ylabel('(A)'); legend('i_d','i_q','Location','best');
    xlim([0 tD(end)]);
    fprintf('  iq    %.0f A peak, id %.2f A max (should stay near zero)\n', max(abs(Id(:,2))), max(abs(Id(:,1))));
end

function shade_one(ax, xr, c)
    yl = ylim(ax); ylim(ax, yl);
    h = patch(ax, [xr(1) xr(2) xr(2) xr(1)], [yl(1) yl(1) yl(2) yl(2)], c, ...
              'FaceAlpha', 0.25, 'EdgeColor','none');
    uistack(h,'bottom'); h.Annotation.LegendInformation.IconDisplayStyle = 'off';
end
