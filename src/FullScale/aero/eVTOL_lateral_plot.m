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

