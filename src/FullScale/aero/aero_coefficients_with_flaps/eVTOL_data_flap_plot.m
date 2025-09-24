
% ===================== LOAD DATA =====================
% High-lift (airfoil: S1223)
[alpha_h_0, CL_h_0, CD_h_0, CM_h_0] = eVTOL_data_alpha_highlift_flap0();
[alpha_h_20, CL_h_20, CD_h_20, CM_h_20] = eVTOL_data_alpha_highlift_flap20();
[alpha_h_30, CL_h_30, CD_h_30, CM_h_30] = eVTOL_data_alpha_highlift_flap30();


% Low-lift (airfoil: NACA23013)
[alpha_l_0, CL_l_0, CD_l_0, CM_l_0] = eVTOL_data_alpha_lowlift_flap0();
[alpha_l_20, CL_l_20, CD_l_20, CM_l_20] = eVTOL_data_alpha_lowlift_flap20();
[alpha_l_30, CL_l_30, CD_l_30, CM_l_30] = eVTOL_data_alpha_lowlift_flap30();

% ===================== PLOTS =====================

% --- Lift curve ---
figure;
plot(alpha_l_0, CL_l_0, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(alpha_l_20, CL_l_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013');
plot(alpha_l_30, CL_l_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
plot(alpha_h_0, CL_h_0, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 0° S1223');
plot(alpha_h_20, CL_h_20, 'm-d', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 20° S1223');
plot(alpha_h_30, CL_h_30, 'k-p', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 30° S1223');
grid on; xlabel('\alpha [deg]'); ylabel('C_L');
title('Lift Curve: High-lift vs Low-lift Airfoils');
legend('Location','best');

% --- Drag polar ---
figure;
plot(alpha_l_0, CD_l_0, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(alpha_l_20, CD_l_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013');
plot(alpha_l_30, CD_l_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
plot(alpha_h_0, CD_h_0, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 0° S1223');
plot(alpha_h_20, CD_h_20, 'm-d', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 20° S1223');
plot(alpha_h_30, CD_h_30, 'k-p', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 30° S1223');
grid on; xlabel('\alpha [deg]'); ylabel('C_D');
title('Drag Polar: High-lift vs Low-lift Airfoils');
legend('Location','best');

% --- Pitching moment ---
figure;
plot(alpha_l_0, CM_l_0, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(alpha_l_20, CM_l_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013');
plot(alpha_l_30, CM_l_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
plot(alpha_h_0, CM_h_0, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 0° S1223');
plot(alpha_h_20, CM_h_20, 'm-d', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 20° S1223');
plot(alpha_h_30, CM_h_30, 'k-p', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 30° S1223');
grid on; xlabel('\alpha [deg]'); ylabel('C_M');
title('Pitching Moment: High-lift vs Low-lift Airfoils');
legend('Location','best');

