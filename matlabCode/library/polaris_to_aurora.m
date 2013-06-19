function Y = polaris_to_aurora(path, H_OT_to_EMT, collectionMethod, recordingType)
% data read in
% do preparation
currentPath = which(mfilename);
if ~exist('path', 'var') || isempty(path)
    pathGeneral = fileparts(fileparts(fileparts(currentPath)));
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];
    testrow_name_EMT = 'hybridEMT';
    testrow_name_OT = 'hybridOT';
elseif strcmp(collectionMethod,'cpp')
    if strcmp(recordingType,'dynamic')
        testrow_name_EMT = 'EMTrackingcont_1';
        testrow_name_OT = 'OpticalTrackingcont_1';
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
        [data_OT, data_EMT, ~,~] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);
        
        % interpolate at e.g. 20Hz, discard .valid == false positions
        
        
        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
        [~, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');
        
        % only take .valid data that were taken at the same time
        [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell);
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
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

Y_tmp = mean(Y_all,3);
Y(:,:) = Y_tmp(:,:,1);

for col = 1:3
Y(1:3,col) = Y(1:3,col) / norm(Y(1:3,col)); 
end

end