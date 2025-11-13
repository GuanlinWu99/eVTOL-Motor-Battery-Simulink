% ===================== LOAD DATA =====================
% High-lift (airfoil: S1223)
[alpha_n5_15_h_30, CL_h_n5_15_30, CD_h_n5_15_30, CM_h_n5_15_30] = eVTOL_data_alpha_n5_15_highlift_flap30();
% Low-lift (airfoil: NACA23013)
[alpha_n5_15_l_30, CL_l_n5_15_30, CD_l_n5_15_30, CM_l_n5_15_30] = eVTOL_data_alpha_n5_15_lowlift_flap30();
% ===================== PLOTS =====================

% --- Lift curve ---
figure;
plot(alpha_n5_15_l_30, CL_l_n5_15_30, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(alpha_n5_15_h_30, CL_h_n5_15_30, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 0° S1223');
grid on; xlabel('\alpha [deg]'); ylabel('C_L');
title('Lift Curve: High-lift vs Low-lift Airfoils');
legend('Location','best');

% --- Drag polar ---
figure;
plot(alpha_n5_15_l_30, CD_l_n5_15_30, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(alpha_n5_15_h_30, CD_h_n5_15_30, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 0° S1223');
grid on; xlabel('\alpha [deg]'); ylabel('C_D');
title('Drag Polar: High-lift vs Low-lift Airfoils');
legend('Location','best');

% --- Pitching moment ---
figure;
plot(alpha_n5_15_l_30, CM_l_n5_15_30, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
plot(alpha_n5_15_h_30, CM_h_n5_15_30, 'g-^', 'LineWidth', 1.5, 'DisplayName', 'High-lift Flap 0° S1223');
grid on; xlabel('\alpha [deg]'); ylabel('C_M');
title('Pitching Moment: High-lift vs Low-lift Airfoils');
legend('Location','best');
