%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% topic: extract aerodynamic coefficients from openVSP aero data         %
% author(s): hanhyun, mingun, minhyun                                    %
% description:                                                           %
% 1. read and process static aero coefficients obtained from openVSP     %
%    (longitudinal, alpha, cl, cd, cm)                                   %
% 2. note that force coefficients are w.r.t. wind axes,                  %
%    moment coefficients are w.r.t body axes                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [alpha_lon, flap_lon, CL, CD, CM] = eVTOL_data_lon()
    %.. set up files for aerodynamic data
    csvFileNames    =   {'Data_eVTOL_lon_flap0.csv',
                         'Data_eVTOL_lon_flap20.csv'};

    %.. set keywords and create empty data structure
    keywords    =   {'Alpha', 'CL', 'CDtot', 'CMm'};
    dataMap     =   containers.Map();
    nAlpha      =   length(-5:1:15);                            % [deg] AoA range
    numPoints   =   nAlpha;                                     % [-] # of data points

    for file_idx = 1:size(csvFileNames,1)
        %.. read raw data from csv files
        opts    =   detectImportOptions(csvFileNames{file_idx}, 'NumHeaderLines', 1);
        rawData =   readcell(csvFileNames{file_idx}, opts);

        %.. read data by items
        for k = 1:length(keywords)
            keyword =   keywords{k};
            rowIdx  =   find(strcmp(string(rawData(:, 1)), keyword));
    
            if isempty(rowIdx)
                disp([keyword 'cant find.']);
                continue;
            end
    
            rawRow = rawData(rowIdx,2:end);
            
            if contains(keyword,'CM') && (size(rawRow,1) == 1)
                rawRow = rawRow;                    % [-] moment coeffcients in conventional axis (CMl,CMm,CMn) has only one row
            else
                rawRow = rawRow(numPoints+1,:);     % [-] read summary aero data (not iterative compution results)
            end
    
            numericData = cellfun(@(x) str2double(string(x)), rawRow);
            numericData(isnan(numericData)) = [];
    
            if numel(numericData) ~= numPoints
                error('%s row does not contain exactly %d numeric values.', keyword, numPoints);
            end
            
            %.. save read data to data structure
            if isKey(dataMap, keyword)
                dataMap(keyword) = [dataMap(keyword); numericData];
            else
                dataMap(keyword) = numericData;
            end
        end
    end

    %.. sanity check and final processing
    for row_idx = 2:size(dataMap('Alpha'),1)
        temp_alpha    =   dataMap('Alpha');
        if temp_alpha(row_idx,:) ~= temp_alpha(1,:)
            error('data sanity check failed: check AoA analysis points of %s', csvFileNames{row_idx});
        end
    end

    for row_idx = 1:size(dataMap('Alpha'),1)
        temp_file_name  =   csvFileNames{row_idx};
        tokens          =   regexp(temp_file_name, 'flap\s*([+-]?\d+(?:p\d+)?)\s*(?:deg)?', 'tokens', 'once');
        if isempty(tokens)
            flap_deg    =   NaN;
        else
            flap_deg    =   str2double(tokens{1});
        end

        if isKey(dataMap, 'Flap')
            dataMap('Flap') = [dataMap('Flap'); flap_deg];
        else
            dataMap('Flap') = flap_deg;
        end
    end

    alpha_lon   =   dataMap('Alpha');
    alpha_lon   =   alpha_lon(1,:);
    flap_lon    =   dataMap('Flap');
    CL          =   dataMap('CL')';
    CD          =   dataMap('CDtot')';
    CM          =   dataMap('CMm')';

end