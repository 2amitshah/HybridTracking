function [Y,H_OT_to_EMT] = polaris_to_aurora_absor(path, H_OT_to_EMT, collectionMethod, recordingType, verbosity, TSOption)
% data read in
% do preparation

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
elseif strcmp(collectionMethod,'cpp') && (~exist(testrow_name_EMT, 'var') || ~exist(testrow_name_OT, 'var'))
    if strcmp(recordingType,'dynamic')
         testrow_name_EMT = 'EMTracking_newsd2';
         testrow_name_OT = 'OpticalTracking_newsd2';
    elseif strcmp(recordingType,'static')
        testrow_name_EMT = 'EMTracking_';
        testrow_name_OT = 'OpticalTracking_';
    end
elseif strcmp(collectionMethod,'ndi') && (~exist(testrow_name_EMT, 'var') || ~exist(testrow_name_OT, 'var'))
    testrow_name_EMT = 'EM_';
    testrow_name_OT = 'OT_';
end


% switch inputdata cases
if strcmp(collectionMethod,'ndi') && strcmp(recordingType,'static')
    % measurements from ndi
    [data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
    [data_OT] = read_NDI_tracking_files(path, testrow_name_OT);
    
    [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'NDIQuat');
    [H_OT_to_OCS_cell, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');
    
    [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell);
    H_OCS_to_OT = H_OCS_to_OT_cell{1};
    
elseif strcmp(collectionMethod,'cpp')
    %measurements form trackingfusion
    if strcmp(recordingType,'static')
        [data_OT, data_EMT, ~,~] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);

        [H_EMT_to_EMCS_cell, ~] = trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
        [H_OT_to_OCS_cell, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');

        [H_EMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity);
        H_OCS_to_OT = H_OCS_to_OT_cell{1};
    elseif strcmp(recordingType,'dynamic')
        % create OT and EMT at interpolated timestamps at 20Hz
        [~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT, testrow_name_OT, 20, verbosity, TSOption);
       
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
    error('input paramaters collectionMethod and recordingType do not match. NDI and dynamic is e.g. not allowed.')
end

numPts = size(H_EMT_to_EMCS,3);
Y_all = zeros(4,4,numPts);
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * (H_OT_to_EMT * H_OCS_to_OT(:,:,i));
end

% averaging to find Y -> temporarily replaced by filtered median. which one
% performs better?
% Y = mean_transformation(Y_all);

% Quality of first guessed Y
if strcmp(verbosity,'vRelease')
    disp('standard deviation of Y position before point-fit')
    disp(std(Y_all(3,4,:)))
end

% median to find Y
% all indices in which polaris system is at least above aurora system
negative_z_bool_indices = Y_all(3,4,:)<0;
median_of_z = median(Y_all(3,4,negative_z_bool_indices));
median_bool_indices = Y_all(3,4,:)<(median_of_z+2) & Y_all(3,4,:)>(median_of_z-2);

if ~any(median_bool_indices)
    error('polaris_to_aurora: heights of polaris differ more than 2mm from mean height, change tolerance or algorithm altogether')
end
Y = mean_transformation(Y_all(:,:,median_bool_indices));

pointSetOTByEMT = zeros(3,numPts);
pointSetOT = zeros(3,numPts);
for i = 1:numPts
   tmp_point_OT_by_EMT = H_EMT_to_EMCS_cell{1}(:,:,i) * H_OT_to_EMT;
   pointSetOTByEMT(:,i) = tmp_point_OT_by_EMT(1:3,4);
   tmp_point_OT = Y * H_OT_to_OCS_cell{1}(:,:,i);
   pointSetOT(:,i) = tmp_point_OT(1:3,4);
end
T = absor(pointSetOT,pointSetOTByEMT);

if strcmp(verbosity,'vDebug')
    disp 'Correction of Y matrix by point-fit:'
    disp(T.M)
end

if strcmp(verbosity,'vDebug')
    H_OT_to_EMCS = zeros(4,4,numPts);
    for i = 1:numPts
        H_OT_to_EMCS(:,:,i) = Y * H_OT_to_OCS_cell{1}(:,:,i);
    end
    wrapper{1}=H_OT_to_EMCS;
    absorfigure = Plot_points(wrapper, [], 1); 
end

Y = T.M * Y;

if strcmp(verbosity,'vDebug')
    H_OT_to_EMCS = zeros(4,4,numPts);
    for i = 1:numPts
        H_OT_to_EMCS(:,:,i) = Y * H_OT_to_OCS_cell{1}(:,:,i);
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
if strcmp(verbosity,'vRelease')
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