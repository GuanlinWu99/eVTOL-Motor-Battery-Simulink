%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% topic: extract aerodynamic coefficients from openVSP aero data         %
% author(s): hanhyun, mingun, minhyun                                    %
% description:                                                           %
% 1. read and process static aero coefficients obtained from openVSP     %
%    (lateral/directional, beta, cy, cr, cn)                             %
% 2. note that force coefficients are w.r.t. wind axes,                  %
%    moment coefficients are w.r.t body axes                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Beta, CY, CR, CN] = eVTOL_data_beta()
    csvFileName =   'Data_eVTOL_beta.csv';

    opts    =   detectImportOptions(csvFileName, 'NumHeaderLines', 1);
    rawData =   readcell(csvFileName, opts);

    keywords    =   {'Beta', 'CS', 'CMx', 'CMy', 'CMz'};

    keywordNames    =   {};
    meanValues      =   [];

    for k = 1:length(keywords)
        keyword =   keywords{k};
        rowIdx  =   find(strcmp(string(rawData(:, 1)), keyword));

        if isempty(rowIdx)
            disp([keyword 'cant find.']);
            continue;
        end

        for r = 1:length(rowIdx)
            thisRow     =   rawData(rowIdx(r), 2:end);  
            numericData =   cellfun(@(x) str2double(string(x)), thisRow);
            rowMean     =   mean(numericData, 'omitnan');
            keywordNames{end+1} =   keyword; 
            meanValues(end+1)   =   rowMean; 
        end
    end

    T   =   table(keywordNames', meanValues', 'VariableNames', {'Keyword', 'MeanValue'});

    numberofpoints  =   11; 

    beta_row    =   strcmp(T.Keyword, 'Beta');
    CMx_rows    =   strcmp(T.Keyword, 'CMx');
    CMy_rows    =   strcmp(T.Keyword, 'CMy');
    CMz_row     =   strcmp(T.Keyword, 'CMz');
    CS_row      =   strcmp(T.Keyword, 'CS');

    beta_ex     =   T.MeanValue(beta_row);
    CMx_ex      =   T.MeanValue(CMx_rows);
    CMy_ex      =   T.MeanValue(CMy_rows);
    CMz_ex      =   T.MeanValue(CMz_row);
    CFy_ex      =   T.MeanValue(CS_row);
   
    Beta        =   beta_ex(1:numberofpoints);
    CY          =   CFy_ex(1:numberofpoints);
    CR          =   CMx_ex(1:numberofpoints).*cos(beta_ex(1:numberofpoints)/180*pi)+CMy_ex(1:numberofpoints).*sin(beta_ex(1:numberofpoints)/180*pi);
    CN          =   CMz_ex(1:numberofpoints);
end

