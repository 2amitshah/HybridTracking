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
H_OT_to_OCS_cell{1} = frame_interpolation(data_OT, [startTime endTime], frequencyHz, 'cpp', TSOption);
H_OT_to_OCS = H_OT_to_OCS_cell{1};

numPts = size(H_OT_to_OCS_cell{1}, 3);
data_OT_common = cell(numPts,1);
for i=1:numPts
    data_OT_common{i}.position = H_OT_to_OCS(1:3,4,i)';
    data_OT_common{i}.orientation = rot2quat(H_OT_to_OCS(1:3,1:3,i))';
    data_OT_common{i}.valid = H_OT_to_OCS(4,4,i);
    if strcmp(TSOption, 'network')
        data_OT_common{i}.TimeStamp = timestampsNewVector(i);
    elseif strcmp(TSOption, 'device')
        data_OT_common{i}.DeviceTimeStamp = timestampsNewVector(i);
    end
end

numSen = size(data_EMT,2);
H_EMT_to_EMCS_cell = cell(1,numSen);
for j=1:numSen
H_EMT_to_EMCS_cell{j} = frame_interpolation(data_EMT(:,j), [startTime endTime], frequencyHz, 'cpp', TSOption);
end

% create 4x4xN matrix for each Sensor, store them in a cell
% [H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT_interpolated, 'CppCodeQuat');

[H_commonEMT_to_EMCS, H_EMCS_to_commonEMT, data_EM_common] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity, 'CppCodeQuat');

% numPts = size(H_commonEMT_to_EMCS,3);
for i=1:numPts
    if strcmp(TSOption, 'network')
        data_EM_common{i}.TimeStamp = timestampsNewVector(i);
    elseif strcmp(TSOption, 'device')
        data_EM_common{i}.DeviceTimeStamp = timestampsNewVector(i);
    end
end

end