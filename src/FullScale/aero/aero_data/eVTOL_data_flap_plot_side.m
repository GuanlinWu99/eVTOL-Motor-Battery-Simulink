clear; clc; close all;

[alpha_latdir, beta_latdir, CS, CR, CN] = Data_eVTOL_beta_AOA0();
colors = lines(length(alpha_latdir));

%% ---- Side force coefficient (C_S)
fig1 = figure('Color','w');
hold on; grid on;
for i = 1:length(alpha_latdir)
    plot(beta_latdir, CS(i,:), '-o', 'LineWidth',1.4, 'Color',colors(i,:));
end
xlabel('\beta [deg]');
ylabel('C_S');
title('Side Force Coefficient vs \beta');
legend(arrayfun(@(a) sprintf('\\alpha = %g°', a), alpha_latdir, 'UniformOutput', false), ...
       'Location','best');
saveas(fig1, 'CS_vs_beta.png');

%% ---- Rolling moment coefficient (C_R)
fig2 = figure('Color','w');
hold on; grid on;
for i = 1:length(alpha_latdir)
    plot(beta_latdir, CR(i,:), '-o', 'LineWidth',1.4, 'Color',colors(i,:));
end
xlabel('\beta [deg]');
ylabel('C_R');
title('Rolling Moment Coefficient vs \beta');
legend(arrayfun(@(a) sprintf('\\alpha = %g°', a), alpha_latdir, 'UniformOutput', false), ...
       'Location','best');
saveas(fig2, 'CR_vs_beta.png');

%% ---- Yawing moment coefficient (C_N)
fig3 = figure('Color','w');
hold on; grid on;
for i = 1:length(alpha_latdir)
    plot(beta_latdir, CN(i,:), '-o', 'LineWidth',1.4, 'Color',colors(i,:));
end
xlabel('\beta [deg]');
ylabel('C_N');
title('Yawing Moment Coefficient vs \beta');
legend(arrayfun(@(a) sprintf('\\alpha = %g°', a), alpha_latdir, 'UniformOutput', false), ...
       'Location','best');
saveas(fig3, 'CN_vs_beta.png');