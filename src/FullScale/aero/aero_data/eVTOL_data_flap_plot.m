% ===================== LOAD DATA =====================
[alpha_0, CL_0, CD_0, CM_0] = Data_eVTOL_alpha_flap0();
[alpha_10, CL_10, CD_10, CM_10] = Data_eVTOL_alpha_flap10();
[alpha_20, CL_20, CD_20, CM_20] = Data_eVTOL_alpha_flap20();
[alpha_30, CL_30, CD_30, CM_30] = Data_eVTOL_alpha_flap30();
[alpha_40, CL_40, CD_40, CM_40] = Data_eVTOL_alpha_flap40();

% ===================== PLOTS =====================

% --- Lift curve ---
fig1 = figure('Color','w');
plot(alpha_0, CL_0, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'Flap : 0°'); hold on
plot(alpha_10, CL_10, 'm-d', 'LineWidth', 1.5, 'DisplayName', 'Flap : 10°');
plot(alpha_20, CL_20, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Flap : 20°');
plot(alpha_30, CL_30, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Flap : 30°');
plot(alpha_40, CL_40, 'k-d', 'LineWidth', 1.5, 'DisplayName', 'Flap : 40°');
grid on; xlabel('\alpha [deg]'); ylabel('C_L');
title('Lift Curve: High-lift vs Low-lift Airfoils');
legend('Location','best');
saveas(fig1, 'LiftCurve.png');

% --- Drag polar ---
fig2 = figure('Color','w');
plot(alpha_0, CD_0, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'Flap 0°'); hold on
plot(alpha_10, CD_10, 'm-d', 'LineWidth', 1.5, 'DisplayName', 'Flap 10°');
plot(alpha_20, CD_20, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Flap 20°');
plot(alpha_30, CD_30, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Flap 30°');
plot(alpha_40, CD_40, 'k-d', 'LineWidth', 1.5, 'DisplayName', 'Flap 40°');
grid on; xlabel('\alpha [deg]'); ylabel('C_D');
title('Drag Polar: High-lift vs Low-lift Airfoils');
legend('Location','best');
saveas(fig2, 'DragPolar.png');

% --- Pitching moment ---
fig3 = figure('Color','w');
plot(alpha_0, CM_0, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'Flap 0°'); hold on
plot(alpha_10, CM_10, 'm-d', 'LineWidth', 1.5, 'DisplayName', 'Flap 10°');
plot(alpha_20, CM_20, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Flap 20°');
plot(alpha_30, CM_30, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Flap 30°');
plot(alpha_40, CM_40, 'k-d', 'LineWidth', 1.5, 'DisplayName', 'Flap 40°');
grid on; xlabel('\alpha [deg]'); ylabel('C_M');
title('Pitching Moment: High-lift vs Low-lift Airfoils');
legend('Location','best');
saveas(fig3, 'PitchingMoment.png');
