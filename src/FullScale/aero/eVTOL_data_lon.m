%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% topic: extract aerodynamic coefficients from openVSP aero data         %
% author(s): hanhyun, mingun, minhyun                                    %
% description:                                                           %
% 1. read and process static aero coefficients obtained from openVSP     %
%    (longitudinal, alpha, cl, cd, cm)                                   %
% 2. note that force coefficients are w.r.t. wind axes,                  %
%    moment coefficients are w.r.t body axes                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [alpha_lon, CL, CD, CM] = eVTOL_data_lon()
    csvFileName =   'Data_eVTOL_lon.csv';

    opts    =   detectImportOptions(csvFileName,'NumHeaderLines',1);
    rawData =   readcell(csvFileName,opts);

    keywords    =   {'Alpha', 'CL', 'CDtot', 'CMm'};
    dataMap     =   containers.Map();

    nAlpha      =   length(-5:1:15);
    nBeta       =   length(0);
    numPoints   =   nAlpha*nBeta;

    keywordNames    =   {};
    meanValues      =   [];

    for k = 1:length(keywords)
        keyword = keywords{k};
        rowIdx = find(strcmp(string(rawData(:,1)), keyword));
        
        if isempty(rowIdx)
            disp([keyword ' not found.']);
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

    alpha_lon   =   reshape(dataMap('Alpha'), [nAlpha, nBeta]);
    CL          =   reshape(dataMap('CL'), [nAlpha, nBeta]);
    CD          =   reshape(dataMap('CDtot'), [nAlpha, nBeta]);
    CM          =   reshape(dataMap('CMm'),  [nAlpha, nBeta]);

end