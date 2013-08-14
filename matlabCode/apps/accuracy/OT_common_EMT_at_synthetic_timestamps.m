%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to one EMT and one OT dataset,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT.
%All available information of all available EM sensors is used to compute
%the position and orientation of the first EM sensor in the best possible
%way. Its position and orientation are computed at synthetic timestamps,
%starting when both EM and OT have values and ending when both stop having
%information. The OT data is computed at these synthetic timestamps as
%well. The results will be in data_EM_common and data_OT_common in the end
%with both having positions and orientations at the same timestamps in a
%given frequency
%
%INPUT
%path: path of the data to be read in
%
%testrow_name_EM: name of the file containing EM data
%
%testrow_name_OT: name of the file containing OT data
%
%frequencyHz: frequency at which timestamps and data will be interpolated
%
%verbosity: level of how many plots are generated
%
%OUTPUT
%H_commonEMT_to_EMCS: contains matrices with the computed EM data of one sensor
%
%H_EMCS_to_commonEMT: contains the inverse matrices of the H_commonEMT_to_EMCS
%
%data_EM_common: contains the computed EM data at the position of EM sensor
%                1 at the synthetic timestamps
%
%data_OT_common: contains the computed OT data at the synthetic timestamps
%
%%%%%%%%%%%%% Authors: Nicola Leucht, Santiago Pérez, Felix Achilles
%%%%%%%%%%%%% June 2013

function [H_commonEMT_to_EMCS, H_EMCS_to_commonEMT, data_EM_common, data_OT_common] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EM, testrow_name_OT, frequencyHz, verbosity, TSOption)
%% OT_common_EMT_at_synthetic_timestamps should be located in \apps\accuracy\

% data read in
% do preparation

% close all;

if ~exist(TSOption, 'var') || isempty(TSOption)
    TSOption = 'network';
end

if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
end

if ~exist('frequencyHz', 'var')
    frequencyHz = 20;
end

if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];

end
if ~exist('testrow_name_EM', 'var')
    testrow_name_EM = 'EMTrackingcont_1';
end
if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OpticalTrackingcont_1';
end

% get data 
[data_OT, data_EMT] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EM, 1);
% determine earliest and latest common timestamp
[interval] = obtain_boundaries_for_interpolation(data_OT, data_EMT, TSOption);
startTime = interval(1);
endTime = interval(2);

if strcmp(TSOption, 'network')
    step = round(1e9/frequencyHz);  % nanoseconds
elseif strcmp(TSOption, 'device')
    step = round(1/frequencyHz);    % seconds
end

%set up desired timestamps
timestampsNewVector = startTime:step:endTime;

% create data_OT_common as an interpolation at defined timestamps
H_OT_to_OCS_cell{1} = frame_interpolation(data_OT, [startTime endTime], frequencyHz, 'cpp');
H_OT_to_OCS = H_OT_to_OCS_cell{1};

numPts = size(H_OT_to_OCS_cell{1}, 3);
data_OT_common = cell(numPts,1);
for i=1:numPts
    data_OT_common{i}.position = H_OT_to_OCS(1:3,4,i)';
    data_OT_common{i}.orientation = rot2quat(H_OT_to_OCS(1:3,1:3,i))';
    data_OT_common{i}.valid = H_OT_to_OCS(4,4,i);
    data_OT_common{i}.TimeStamp = timestampsNewVector(i);
end

%% data_OT_structarray = [data_OT{:}];
% data_OT_timestamps = [data_OT_structarray.TimeStamp];
% 
% % find entries that should be discarded because of missing sensor data
% for i = 1:size(errorTimeStampsOT,1)
%     if ~isempty(errorTimeStampsOT{i})
%         errorTimeStamp = errorTimeStampsOT{i}; 
% 
%         error_index_minusone = find(errorTimeStamp > data_OT_timestamps,1,'last');
%         if ~isempty(error_index_minusone)
%             errorTimeMin = data_OT_timestamps(error_index_minusone);
%             if error_index_minusone ~= length(data_OT_timestamps)
%                 errorTimeMax = data_OT_timestamps(error_index_minusone+1);
%             else
%                 errorTimeMax = errorTimeStamp;
%             end
%         else
%             errorTimeMin = errorTimeStamp;
%             errorTimeMax = errorTimeStamp;
%         end
%         
%         posMin = floor((errorTimeMin - startTime) / stepsize);
%         posMax = ceil((errorTimeMax - startTime) / stepsize);
%         for s=posMin:posMax
%             if(s<=size(data_OT_common,1) && s > 1)
%                 data_OT_common{s} = data_OT_common{s-1}; %copy position before the error to erroneous locations
%             end
%                 data_OT_common{s}.valid = 0;
%         end
%     end
%% end

% data_EMT_interpolated = synthetic_timestamps(data_EMT, [startTime endTime], frequencyHz, 'cpp');
H_EMT_to_EMCS_cell{1} = frame_interpolation(data_EMT(:,1), [startTime endTime], frequencyHz, 'cpp');
H_EMT_to_EMCS_cell{2} = frame_interpolation(data_EMT(:,2), [startTime endTime], frequencyHz, 'cpp');

%% numSen = size(data_EMT,2);
% data_EMT_structarray_cell = cell(1,numSen);
% data_EMT_timestamps_cell = cell(1,numSen);
% for j=1:numSen
%     data_EMT_structarray_cell{j} = [data_EMT{:,j}];
%     data_EMT_timestamps_cell{j} = [data_EMT_structarray_cell{j}.TimeStamp];
% end

% % find entries that should be discarded because of missing sensor data
% for j=1:numSen
%     last_nonzero_index = find(data_EMT_timestamps_cell{j}~=0,1,'last');
%     if size(errorTimeStampsEM, 2) >= j %maybe not all sensors had errors. If j bigger than size(errorTimeStampsEM, 2), size(errorTimeStampsEM(:,j) would give out an matlabError.
%     for i = 1:size(errorTimeStampsEM(:,j),1)
%         if ~isempty(errorTimeStampsEM{i,j})
%             
%             errorTimeStamp = errorTimeStampsEM{i,j};
% 
%             error_index_minusone = find(errorTimeStamp > data_EMT_timestamps_cell{j},1,'last');
%             if ~isempty(error_index_minusone)
%                 errorTimeMin = data_EMT_timestamps_cell{j}(error_index_minusone);
%                 if error_index_minusone ~= last_nonzero_index
%                     errorTimeMax = data_EMT_timestamps_cell{j}(error_index_minusone+1);
%                 else
%                     errorTimeMax = errorTimeStamp;
%                 end
%             else
%                 errorTimeMin = errorTimeStamp;
%                 errorTimeMax = errorTimeStamp;
%             end
% 
%             posMin = floor((errorTimeMin - startTime) / stepsize);
%             posMax = ceil((errorTimeMax - startTime) / stepsize);
% 
%             for s=posMin:posMax
%                 if(s<=size(data_EMT_interpolated,1) && s > 1)
%                     data_EMT_interpolated{s,j} = data_EMT_interpolated{s-1,j}; %copy position before the error to erroneous locations
%                 end
%                 data_EMT_interpolated{s,j}.valid = 0;
%             end
%         end
%     end
%     end
% end

% create 4x4xN matrix for each Sensor, store them in a cell
%% [H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT_interpolated, 'CppCodeQuat');

[H_commonEMT_to_EMCS, H_EMCS_to_commonEMT, data_EM_common] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity, 'CppCodeQuat');

% numPts = size(H_commonEMT_to_EMCS,3);
for i=1:numPts
%     data_EM_common{i}.position = H_commonEMT_to_EMCS(1:3,4,i)';
%     data_EM_common{i}.orientation = rot2quat(H_commonEMT_to_EMCS(1:3,1:3,i))';
%     data_EM_common{i}.valid = H_commonEMT_to_EMCS(4,4,i);
    data_EM_common{i}.TimeStamp = timestampsNewVector(i);
end

end