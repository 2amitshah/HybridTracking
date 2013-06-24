function Y = polaris_to_aurora(path, H_OT_to_EMT, collectionMethod, recordingType)
% data read in
% do preparation
currentPath = which(mfilename);

if ~exist('H_OT_to_EMT', 'var') || isempty(H_OT_to_EMT)
    load(which('H_OT_to_EMT.mat'));
end

if ~exist('collectionMethod', 'var') || isempty(collectionMethod)
    collectionMethod = 'ndi';
end

if ~exist('recordingType', 'var') || isempty(recordingType)
    recordingType = 'static';
end


if ~exist('path', 'var') || isempty(path)
    pathGeneral = fileparts(fileparts(fileparts(currentPath)));
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];
    testrow_name_EMT = 'hybridEMT';
    testrow_name_OT = 'hybridOT';
elseif strcmp(collectionMethod,'cpp')
    if strcmp(recordingType,'dynamic')
        testrow_name_EMT = 'EMTrackingcont_1';
        testrow_name_OT = 'OpticalTrackingcont_1';
% testrow_name_EMT = 'cont_EMTracking';
% testrow_name_OT = 'cont_OpticalTracking';
    elseif strcmp(recordingType,'static')
        testrow_name_EMT = 'EMTracking_';
        testrow_name_OT = 'OpticalTracking_';
    end
elseif strcmp(collectionMethod,'ndi')
    testrow_name_EMT = 'EM_';
    testrow_name_OT = 'OT_';
end
if ~exist('H_OT_to_EMT', 'var') || isempty(H_OT_to_EMT)
    load(which('H_OT_to_EMT.mat'));
end

if ~exist('collectionMethod', 'var') || isempty(collectionMethod)
    collectionMethod = 'ndi';
end

if ~exist('recordingType', 'var') || isempty(recordingType)
    recordingType = 'static';
end

% switch inputdata cases
if strcmp(collectionMethod,'ndi') && strcmp(recordingType,'static')
    % measurements from ndi
    [data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
    [data_OT] = read_NDI_tracking_files(path, testrow_name_OT);
    
    [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'NDIQuat');
    [~, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');
    
    [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell);
    H_OCS_to_OT = H_OCS_to_OT_cell{1};
    
elseif strcmp(collectionMethod,'cpp')
    %measurements form trackingfusion
    if strcmp(recordingType,'static')
        [data_OT, data_EMT, ~,~] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);

        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
        [~, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');

        [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell);
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
    elseif strcmp(recordingType,'dynamic')
        [~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT, testrow_name_OT, 20);
%         [data_OT, data_EMT, ~,~] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);
%         
%         % interpolate at e.g. 20Hz, discard .valid == false positions
%         interval = obtain_boundaries_for_interpolation( data_OT, data_EMT );
%         data_OT = synthetic_timestamps( data_OT, interval, 20);
%         data_EMT = synthetic_timestamps( data_EMT, interval, 20);
        

% read out the .valid parameter and store in array
data_EMT_arraystruct = [data_EMT{:,1}];
data_EMT_valid_array = [data_EMT_arraystruct.valid];
data_OT_arraystruct = [data_OT{:}];
data_OT_valid_array = [data_OT_arraystruct.valid];

% create bool array that marks valid points
OT_and_EMT_valid = data_EMT_valid_array & data_OT_valid_array;
% update struct cells %DEBUG!
data_OT_arraystruct = [data_OT{OT_and_EMT_valid}]';
data_EMT_arraystruct = [data_EMT{OT_and_EMT_valid}]';
data_OT = num2cell(data_OT_arraystruct);
data_EMT = num2cell(data_EMT_arraystruct);
% save memory
clear data_EMT_arraystruct data_OT_arraystruct
        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
        [~, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');
%         
%         % only take .valid data that were taken at the same time
%         [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell);
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
        H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
    end
    
else
    error('input paramaters collectionMethod and recordingType do not match. NDI and dynamic is e.g. not allowed.')
end

% averaging to find Y
numPts = size(H_EMT_to_EMCS,3);
Y_all = zeros(4,4,numPts);
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * H_OT_to_EMT * H_OCS_to_OT(:,:,i);
end
%TODO: for whatever reason, like half of the transformations give e.g.
%positive Z-values, which is impossible beacause the Polaris device is
%always above (-Z means up in Aurora System) the Field Generator
%Only frames with a negative Z value are taken. Very ugly but this way it
%works...
Y = mean_transformation(Y_all(:,:,[Y_all(3,4,:)<0]));
% Y = mean_transformation(Y_all);


end