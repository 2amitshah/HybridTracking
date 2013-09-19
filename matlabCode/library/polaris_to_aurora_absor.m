function [Y,AbsorError] = polaris_to_aurora_absor(path, H_OT_to_EMT, collectionMethod, recordingType, verbosity, TSOption)
%% data read in


filenames_struct = path;
if isstruct(filenames_struct)
    testrow_name_EMT = filenames_struct.EMfiles;
    testrow_name_OT = filenames_struct.OTfiles;
    path = filenames_struct.folder;
else
    warning('GeneralWarning:pathStruct',['Please use the new filenames_struct-feature.\n'...
        ' ''path'' can now be a struct, so you don''t always have to change the default ''testrow_name_EMT'' and ''testrow_name_OT''.'])
end

if ~exist('TSOption', 'var') || isempty(TSOption)
    TSOption = 'network';
end

if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
end

if ~exist('H_OT_to_EMT', 'var') || isempty(H_OT_to_EMT)
    load('H_OT_to_EMT.mat');
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
elseif strcmp(collectionMethod,'cpp') && (~exist('testrow_name_EMT', 'var') || ~exist('testrow_name_OT', 'var'))
    if strcmp(recordingType,'dynamic')

    elseif strcmp(recordingType,'static')
        testrow_name_EMT = 'EMTracking_';
        testrow_name_OT = 'OpticalTracking_';
    end
elseif strcmp(collectionMethod,'ndi') && (~exist('testrow_name_EMT', 'var') || ~exist('testrow_name_OT', 'var'))
    testrow_name_EMT = 'EM_';
    testrow_name_OT = 'OT_';
end


% switch inputdata cases
if strcmp(collectionMethod,'ndi')
    % measurements from ndi
    if strcmp(recordingType,'static')
        [data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
        [data_OT] = read_NDI_tracking_files(path, testrow_name_OT);

        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'NDIQuat');
        [H_OT_to_OCS_cell, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');

        [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity, 'ndi');
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
    elseif strcmp(recordingType,'dynamic')
        % read data
        [data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT, 'dynamic', 'Aurora');
        [data_OT] = read_NDI_tracking_files(path, testrow_name_OT, 'dynamic', 'Polaris');
        
        % synchronize
        EM_minus_OT_offset = sync_from_file(filenames_struct, 'vRelease', 'device', collectionMethod);
        numPtsEMT = size(data_EMT,1);
        numEMs = size(data_EMT,2);
        for i = 1:numPtsEMT
            for j = 1:numEMs
                if ~isempty(data_EMT{i,j})
                    % move EM timestamps into timeframe of Optical (because Optical is our common reference)
                    data_EMT{i,j}.DeviceTimeStamp = data_EMT{i,j}.DeviceTimeStamp - EM_minus_OT_offset;
                end
            end
        end
        
        % interpolate at simultaneous timestamps
        interval = obtain_boundaries_for_interpolation(data_OT, data_EMT, 'device');
        H_EMT_to_EMCS_cell = cell(1,numEMs);
        for j = 1:numEMs
            [H_EMT_to_EMCS_cell{j}] = frame_interpolation(data_EMT(:,j), interval, 60, collectionMethod, 'device');
        end
        [H_OT_to_OCS_cell{1}, H_OCS_to_OT_cell{1}] = frame_interpolation(data_OT, interval, 60, collectionMethod, 'device');

        % calculate a common EMT frame
        [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity, 'ndi');
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
    end
    
elseif strcmp(collectionMethod,'cpp')
    %measurements form trackingfusion
    if strcmp(recordingType,'static')
        [data_OT, data_EMT, ~,~] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);

        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
        [H_OT_to_OCS_cell, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');

        [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity,'cpp');
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
    elseif strcmp(recordingType,'dynamic')
        % create OT and EMT at interpolated timestamps at 20Hz
        [~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT, testrow_name_OT, 20, verbosity, TSOption);
% error('OT_common_EMT_at_synthetic_timestamps with TSOption = device does not synchronize the streams before interpolation. Very Bad. TODO!')
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
        [H_OT_to_OCS_cell, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');         
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
        H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
    end
    
else
    error('input paramaters collectionMethod and recordingType do not match.')
end
%% average to find Y
numPts = size(H_EMT_to_EMCS,3);
Y_all = zeros(4,4,numPts);
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * (H_OT_to_EMT * H_OCS_to_OT(:,:,i));
end

% averaging to find Y -> temporarily replaced by filtered median. which one
% performs better?
% Y = mean_transformation(Y_all);

%% Quality of first guessed Y
if strcmp(verbosity,'vRelease') || strcmp(verbosity,'vDebug')
    disp('standard deviation of Y position before point-fit')
    disp(std(Y_all(3,4,:)))
end
if  strcmp(verbosity,'vDebug')
    zOfY_Figure = figure;
    plot(permute(Y_all(3,4,:),[3 2 1]))
end

%% median to find Y
% all indices in which polaris system is at least above aurora system
negative_z_bool_indices = Y_all(3,4,:)<0;
median_of_z = median(Y_all(3,4,negative_z_bool_indices));
median_bool_indices = Y_all(3,4,:)<(median_of_z+2) & Y_all(3,4,:)>(median_of_z-2);

if ~any(median_bool_indices)
    error('polaris_to_aurora: heights of polaris differ more than 2mm from mean height, change tolerance or algorithm altogether')
end
Y = mean_transformation(Y_all(:,:,median_bool_indices));

if strcmp(verbosity,'vRelease') || strcmp(verbosity,'vDebug')
    disp('standard deviation of Y position after median filtering but before point-fit')
    disp(std(Y_all(3,4,median_bool_indices)))
end

%% Horns method to improve Y
pointSetOTByEMT = zeros(3,numPts);
pointSetOT = zeros(3,numPts);
for i = 1:numPts
   tmp_point_OT_by_EMT = H_EMT_to_EMCS_cell{1}(:,:,i) * H_OT_to_EMT;
   pointSetOTByEMT(:,i) = tmp_point_OT_by_EMT(1:3,4);
   tmp_point_OT = Y * H_OT_to_OCS_cell{1}(:,:,i);
   pointSetOT(:,i) = tmp_point_OT(1:3,4);
end
[T,~,ErrorStats] = absor(pointSetOT,pointSetOTByEMT);
AbsorError = ErrorStats.errlsq / sqrt(numPts);
if strcmp(verbosity,'vDebug')
    disp 'Correction of Y matrix by point-fit:'
    disp(T.M)
end
disp 'Remaining RMS translation error of Y:'
disp(AbsorError)

if strcmp(verbosity,'vDebug')
    H_OT_to_EMCS = zeros(4,4,numPts);
    for i = 1:numPts
        H_OT_to_EMCS(:,:,i) = Y * H_OT_to_OCS_cell{1}(:,:,i);
    end
    wrapper{1}=H_OT_to_EMCS;
    absorfigure = Plot_points(wrapper, [], 1); 
end

Yh = T.M * Y;

if strcmp(verbosity,'vDebug')
    H_OT_to_EMCS = zeros(4,4,numPts);
    for i = 1:numPts
        H_OT_to_EMCS(:,:,i) = Yh * H_OT_to_OCS_cell{1}(:,:,i);
    end
    wrapper{1}=H_OT_to_EMCS;
    Plot_points(wrapper, absorfigure, 2); 

    OT_by_EMT = zeros(4,4,numPts);
    for i = 1:numPts
        OT_by_EMT(:,:,i) = H_EMT_to_EMCS_cell{1}(:,:,i) * H_OT_to_EMT;
    end
    wrapper{1}=OT_by_EMT;
    Plot_points(wrapper, absorfigure, 3); 
    plotAuroraTable(absorfigure);
    title('polaris\_to\_aurora\_absor: Blue is OT before ICP, green is optical after ICP and red is target (OT by EMT)');
end

%% ICP to improve Horn
[R, T, Err] = icp(pointSetOTByEMT, pointSetOT);
H_icp = [R, T; 0 0 0 1];

Yi = H_icp*Y;

disp('RMS Error of ICP')
disp(Err(end))

if strcmp(verbosity,'vDebug')
    H_OT_to_EMCS = zeros(4,4,numPts);
    for i = 1:numPts
        H_OT_to_EMCS(:,:,i) = Yi * H_OT_to_OCS_cell{1}(:,:,i);
    end
    wrapper{1}=H_OT_to_EMCS;
    Plot_points(wrapper, absorfigure, 4); 

    title('polaris\_to\_aurora\_absor: Blue is OT before ICP, green/turquoise is optical after ICP and red is target (OT by EMT)');
end

% choose ICP corrected transformation if it meant an improvement
if Err(end) < rms(Y_all(3,4,median_bool_indices))
    Y = Yi;
    AbsorError = Err(end);
else
    AbsorError = rms(Y_all(3,4,median_bool_indices));
end

%% improvement of X (didn't work so far)
%     %X_err * OT_to_EMT * OCS_to_OT = EMCS_to_EMT * Y
%     pointSetOT_to_EMT_OCS_to_OT = zeros(3,numPts);
%     pointSetEMCS_to_EMT_Y = zeros(3,numPts);
%     for i = 1:numPts
%         tmpPt = H_OT_to_EMT * H_OCS_to_OT_cell{1}(:,:,i);
%         pointSetOT_to_EMT_OCS_to_OT(:,i) = tmpPt(1:3,4);
%         tmpPt2 = H_EMCS_to_EMT_cell{1}(:,:,i) * Y;
%         pointSetEMCS_to_EMT_Y(:,i) = tmpPt2(1:3,4);
%     end
%     T = absor(pointSetOT_to_EMT_OCS_to_OT,pointSetEMCS_to_EMT_Y);
%     H_OT_to_EMT = T.M * H_OT_to_EMT;
% end
%%

% plot position data
if strcmp(verbosity,'vRelease') || strcmp(verbosity,'vDebug')
    plothandle=Plot_points(H_EMT_to_EMCS_cell, [], 1); % common EMT is blue
    H_OT_to_EMCS = zeros(4,4,numPts);
    for i = 1:numPts
        H_OT_to_EMCS(:,:,i) = Y * H_OT_to_OCS_cell{1}(:,:,i);
    end
    wrapper{1}=H_OT_to_EMCS;
    Plot_points(wrapper, plothandle, 3); %optical is red
    plotEnvironment(plothandle, H_OT_to_EMT, Y)
    title('polaris\_to\_aurora\_absor: Position of common EMT1 sensor (blue) and optical sensor (red) at synchronous timestamps')
end

end