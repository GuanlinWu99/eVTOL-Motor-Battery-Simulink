%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% topic: extract aerodynamic coefficients from openVSP aero data         %
% author(s): hanhyun, mingun, minhyun                                    %
% description:                                                           %
% 1. read and process static aero coefficients obtained from openVSP     %
%    (lateral/directional, beta, cy, cr, cn)                             %
% 2. note that force coefficients are w.r.t. wind axes,                  %
%    moment coefficients are w.r.t body axes                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [alpha_latdir, beta_latdir, CS, CR, CN] = eVTOL_data_latdir()
    csvFileName =   'Data_eVTOL_latdir.csv';

    opts    =   detectImportOptions(csvFileName, 'NumHeaderLines', 1);
    rawData =   readcell(csvFileName, opts);

    keywords    =   {'Alpha', 'Beta', 'CS', 'CMl', 'CMn'};
    dataMap     =   containers.Map();

    nAlpha      =   length(0:5:10);
    nBeta       =   length(-10:2:10);
    numPoints   =   nAlpha*nBeta;

    keywordNames    =   {};
    meanValues      =   [];

    for k = 1:length(keywords)
        keyword =   keywords{k};
        rowIdx  =   find(strcmp(string(rawData(:, 1)), keyword));

        if isempty(rowIdx)
            disp([keyword 'cant find.']);
            continue;
        end

        rawRow = rawData(rowIdx,2:end);
        
        if contains(keyword,'CM')
            rawRow = rawRow;
        else
            rawRow = rawRow(numPoints+1,:);
        end

        numericData = cellfun(@(x) str2double(string(x)), rawRow);
        numericData(isnan(numericData)) = [];

        if numel(numericData) ~= numPoints
            error('%s row does not contain exactly %d numeric values.', keyword, numPoints);
        end

        dataMap(keyword) = numericData;
    end

    alpha_latdir    =   reshape(dataMap('Alpha'), [nAlpha, nBeta]);
    alpha_latdir    =   alpha_latdir(:,1);
    beta_latdir     =   reshape(dataMap('Beta'), [nAlpha, nBeta]);
    beta_latdir     =   beta_latdir(1,:)';
    CS              =   reshape(dataMap('CS'), [nAlpha, nBeta]);
    CR              =   reshape(dataMap('CMl'), [nAlpha, nBeta]);
    CN              =   reshape(dataMap('CMn'), [nAlpha, nBeta]);

end

