%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% topic: extract aerodynamic coefficients from openVSP aero data         %
% author(s): hanhyun, mingun, minhyun                                    %
% description:                                                           %
% 1. read and process static aero coefficients obtained from openVSP     %
%    (longitudinal, alpha, cl, cd, cm)                                   %
% 2. note that force coefficients are w.r.t. wind axes,                  %
%    moment coefficients are w.r.t body axes                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [alpha_n5_15_l_0, CL_l_n5_15_0, CD_l_n5_15_0, CM_l_n5_15_0] = eVTOL_data_alpha_n5_15_lowlift_flap30()

    csvFileName =   'Data_eVTOL_alpha_n5_15_lowlift_Flap30.csv';

    opts    =   detectImportOptions(csvFileName, 'NumHeaderLines', 1);
    rawData =   readcell(csvFileName, opts);

    keywords    =   {'Alpha', 'CL', 'CDtot', 'CMy'};

    keywordNames    =   {};
    meanValues      =   [];

    for k = 1:length(keywords)
        keyword = keywords{k};
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

    nAlpha          =   sum(strcmp(T.Keyword, 'Alpha'));
    numberofpoints  =   nAlpha - 1;

    alpha_row   =   strcmp(T.Keyword, 'Alpha');
    cl_rows     =   strcmp(T.Keyword, 'CL');
    CD_tot_row  =   strcmp(T.Keyword, 'CDtot');
    CMy_row     =   strcmp(T.Keyword, 'CMy');

    alpha_ex    =   T.MeanValue(alpha_row);
    CL_ex       =   T.MeanValue(cl_rows);
    CD_ex       =   T.MeanValue(CD_tot_row);
    CMy_ex      =   T.MeanValue(CMy_row);

    alpha_n5_15_l_0       =   alpha_ex(1:numberofpoints);
    CL_l_n5_15_0          =   CL_ex(1:numberofpoints);
    CD_l_n5_15_0          =   CD_ex(1:numberofpoints);
    CM_l_n5_15_0          =   CMy_ex(1:numberofpoints);
end