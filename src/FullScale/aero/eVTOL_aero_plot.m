%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Topic: Plot Lateral Coefficients at alpha, beta                        %
% Author(s): Mingun                                                      %
% Description:                                                           %
% 1.                                                                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

csvFileName = 'Data_eVTOL_latdir.csv';

opts = detectImportOptions(csvFileName, 'NumHeaderLines', 1);
rawData = readcell(csvFileName, opts);

keywords = {'Alpha', 'Beta', 'CMx', 'CMz', 'CS'}; %  
dataMap = containers.Map();

nAlpha = 3;
nBeta  = 11;
numPoints = nAlpha*nBeta; 

for k = 1:length(keywords)
    keyword = keywords{k};
    rowIdx = find(strcmp(string(rawData(:,1)), keyword));
    
    if isempty(rowIdx)
        disp([keyword ' not found.']);
        continue;
    end
    
    rawRow = rawData(rowIdx, 2:end);
    % rawRow = rawRow(1:numPoints);
    rawRow = rawRow(numPoints+1,:);
    numericData = cellfun(@(x) str2double(string(x)), rawRow);
    numericData(isnan(numericData)) = [];

    if numel(numericData) ~= numPoints
        error('%s row does not contain exactly 33 numeric values.', keyword);
    end

    dataMap(keyword) = numericData;
end

alpha_grid = reshape(dataMap('Alpha'), [nAlpha, nBeta]);
beta_grid  = reshape(dataMap('Beta'),  [nAlpha, nBeta]);
cmx = reshape(dataMap('CMx'), [nAlpha, nBeta]);
cmz = reshape(dataMap('CMz'), [nAlpha, nBeta]);
cs  = reshape(dataMap('CS'),  [nAlpha, nBeta]);

plot_coeff(alpha_grid, beta_grid, cmx, 'CMx');
plot_coeff(alpha_grid, beta_grid, cmz, 'CMz');
plot_coeff(alpha_grid, beta_grid, cs,  'CS');

function plot_coeff(alpha, beta, coeffMat, coeffLabel)
    figure;

    x = alpha(:);
    y = beta(:);
    z = coeffMat(:);

    scatter3(x, y, z, 50, 'MarkerFaceColor', 'b');

    xlabel('Alpha');
    ylabel('Beta');
    zlabel(coeffLabel);
    title(coeffLabel);
    grid on;
    view(45, 30);
    hold off;
end

% CY = CS / CI = CMx / Cn = CMz

% figure('Units','normalized','OuterPosition',[0.1 0.1 0.4 0.25], 'Color','w'); 
% subplot(1,3,1)
% plot(uavParams.aero.alpha_lon*180/pi,uavParams.aero.CL,'LineWidth',2); grid on;
% xlabel('AoA')
% ylabel('(-)')
% title('CL')
% 
% subplot(1,3,2)
% plot(uavParams.aero.alpha_lon*180/pi,uavParams.aero.CD,'LineWidth',2); grid on;
% xlabel('AoA')
% ylabel('(-)')
% title('CD')
% 
% subplot(1,3,3)
% plot(uavParams.aero.alpha_lon*180/pi,uavParams.aero.CM,'LineWidth',2); grid on;
% xlabel('AoA')
% ylabel('(-)')
% title('CM')
% 
% sgtitle('Aerodynamic Coefficients','FontSize',15,'FontWeight','bold');

%% Check CM, CL, CD

% figure('Units','normalized','OuterPosition',[0.1 0.1 0.5 0.35], 'Color','w'); 
% subplot(1,3,1)
% plot(uavParams.aero.alpha_lon*180/pi, uavParams.aero.CL, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
% plot(uavParams.aero.alpha_lon_20*180/pi, uavParams.aero.CL_flap_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013'); hold on;
% plot(uavParams.aero.alpha_lon_30*180/pi, uavParams.aero.CL_flap_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
% grid on; xlabel('\alpha [deg]'); ylabel('C_L'); title('Lift Curve'); legend('Location','best');
% 
% subplot(1,3,2)
% plot(uavParams.aero.alpha_lon*180/pi, uavParams.aero.CD, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
% plot(uavParams.aero.alpha_lon_20*180/pi, uavParams.aero.CD_flap_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013'); hold on;
% plot(uavParams.aero.alpha_lon_30*180/pi, uavParams.aero.CD_flap_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
% grid on; xlabel('\alpha [deg]'); ylabel('C_D'); title('Drag Curve'); legend('Location','best');
% 
% subplot(1,3,3)
% plot(uavParams.aero.alpha_lon*180/pi, uavParams.aero.CM, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 0° NACA23013'); hold on;
% plot(uavParams.aero.alpha_lon_20*180/pi, uavParams.aero.CM_flap_20, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 20° NACA23013'); hold on;
% plot(uavParams.aero.alpha_lon_30*180/pi, uavParams.aero.CM_flap_30, 'c-*', 'LineWidth', 1.5, 'DisplayName', 'Low-lift Flap 30° NACA23013');
% grid on; xlabel('\alpha [deg]'); ylabel('C_M'); title('Pitching Moment');legend('Location','best');