function Y = polaris_to_aurora(path, H_OT_to_EMT, collectionMethod, recordingType, verbosity)
% data read in
% do preparation

filenames_struct = path;
if isstruct(filenames_struct)
    testrow_name_EMT = filenames_struct.EMfiles;
    testrow_name_OT = filenames_struct.OTfiles;
    path = filenames_struct.folder;
end


if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
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

if ~exist('path', 'var') || isempty(path)
    currentPath = which(mfilename);
    pathGeneral = fileparts(fileparts(fileparts(currentPath)));
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];
    testrow_name_EMT = 'hybridEMT';
    testrow_name_OT = 'hybridOT';
elseif strcmp(collectionMethod,'cpp')
    if strcmp(recordingType,'dynamic')
%         testrow_name_EMT = 'EMTrackingcont_1';
%         testrow_name_OT = 'OpticalTrackingcont_1';
%         testrow_name_EMT = 'cont_EMTracking_3';
%         testrow_name_OT = 'cont_OpticalTracking_3';
% testrow_name_EMT = 'EMTracking_firstVolume';
% testrow_name_OT = 'OpticalTracking_firstVolume';
    elseif strcmp(recordingType,'static')
        testrow_name_EMT = 'EMTracking_';
        testrow_name_OT = 'OpticalTracking_';
    end
elseif strcmp(collectionMethod,'ndi')
    testrow_name_EMT = 'EM_';
    testrow_name_OT = 'OT_';
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
    %measurements from TrackingFusion
    if strcmp(recordingType,'static')
        [data_OT, data_EMT, ~,~] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);
        
        data_EMT_corrected = distortion_correction(data_EMT);
        
        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT_corrected, 'CppCodeQuat');
        [~, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');

        [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity);
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
    elseif strcmp(recordingType,'dynamic')
        % create OT and EMT at interpolated timestamps at 20Hz
        [~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT, testrow_name_OT, 5, verbosity);
       
        % read out the .valid parameter and store in array
        data_EMT_arraystruct = [data_EMT{:,1}];
        data_EMT_valid_array = [data_EMT_arraystruct.valid];
        data_OT_arraystruct = [data_OT{:}];
        data_OT_valid_array = [data_OT_arraystruct.valid];

        % create bool array that marks valid points
        OT_and_EMT_valid = data_EMT_valid_array & data_OT_valid_array;
        
        % update struct cells, only take .valid data that were taken at the same time
        data_OT_arraystruct = [data_OT{OT_and_EMT_valid}]';
        data_EMT_arraystruct = [data_EMT{OT_and_EMT_valid}]';
        data_OT = num2cell(data_OT_arraystruct);
        data_EMT = num2cell(data_EMT_arraystruct);
        
        % clear memory
        clear data_EMT_arraystruct data_OT_arraystruct
        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
        [~, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');         
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
        H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
    end
    
else
    error('input paramaters collectionMethod and recordingType do not match. NDI and dynamic is e.g. not allowed.')
end


numPts = size(H_EMT_to_EMCS,3);
Y_all = zeros(4,4,numPts);
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * (H_OT_to_EMT * H_OCS_to_OT(:,:,i));
end

% averaging to find Y

%TODO: for whatever reason, like half of the transformations give e.g.
%positive Z-values, which is impossible beacause the Polaris device is
%always above (-Z means up in Aurora System) the Field Generator
%Only frames with a negative Z value are taken. Very ugly but this way it
%works...
% Y = mean_transformation(Y_all(:,:,Y_all(3,4,:)<0));
% Y = mean_transformation(Y_all);


% median to find Y
% all indices in which polaris system is at least above aurora system
negative_z_bool_indices = Y_all(3,4,:)<0;
median_of_z = median(Y_all(3,4,negative_z_bool_indices));
median_bool_indices = Y_all(3,4,:)<(median_of_z+2) & Y_all(3,4,:)>(median_of_z-2);
if ~any(median_bool_indices)
    error('polaris_to_aurora: heights of polaris differ more than 2mm from mean height, change tolerance or algorithm altogether')
end
Y = mean_transformation(Y_all(:,:,median_bool_indices));


% plot position data
if strcmp(verbosity,'vDebug')
wrapper{1}=H_EMT_to_EMCS;
plothandle=Plot_points(wrapper, [], 1); %EMT1 is blue
H_OT_to_EMCS = zeros(4,4,numPts);
for i = 1:numPts
    H_OT_to_EMCS(:,:,i) = Y/H_OCS_to_OT(:,:,i);
end
wrapper{1}=H_OT_to_EMCS;
Plot_points(wrapper, plothandle, 3); %optical is blue
plotEnvironment(plothandle, H_OT_to_EMT, Y)
title('Position of common EMT1 sensor (blue) and optical sensor (red) at synchronous timestamps')
end

end