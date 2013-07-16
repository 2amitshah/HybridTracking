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

function [H_commonEMT_to_EMCS, H_EMCS_to_commonEMT, data_EM_common, data_OT_common] = OT_common_EMT_at_synthetic_timestamps_distortion_correction(path, testrow_name_EM, testrow_name_OT, frequencyHz, verbosity)
% OT_common_EMT_at_synthetic_timestamps_distortion_correction should be
% located in \apps\accuracy\

% data read in
% do preparation

filenames_struct = path;
if isstruct(filenames_struct)
    testrow_name_EM = filenames_struct.EMfiles;
    testrow_name_OT = filenames_struct.OTfiles;
    path = filenames_struct.folder;
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
[data_OT, data_EMT, errorTimeStampsOT, errorTimeStampsEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EM);


%% determine earliest and latest common timestamp
[interval] = obtain_boundaries_for_interpolation(data_OT, data_EMT);
startTime = interval(1);
endTime = interval(2);

%set up wished timestamps
stepsize = 1*10^9 / frequencyHz;

% create data_OT_common as an interpolation at defined timestamps, all
% entrys are marked as .valid = 1 as default
data_OT_common = synthetic_timestamps(data_OT, [startTime endTime], frequencyHz, 'cpp');


% I store all timestamps in an array so i can compare to the error
% timestamps and find the predecessor and the successor. Not the finest
% solution but a working one.
data_OT_structarray = [data_OT{:}];
data_OT_timestamps = [data_OT_structarray.TimeStamp];


% find entries that should be discarded because of missing sensor data
for i = 1:size(errorTimeStampsOT,1)
    if ~isempty(errorTimeStampsOT{i})
        errorTimeStamp = errorTimeStampsOT{i};
        error_index_minusone = find(errorTimeStamp > data_OT_timestamps,1,'last');
        if ~isempty(error_index_minusone)
            errorTimeMin = data_OT_timestamps(error_index_minusone);
            if error_index_minusone ~= length(data_OT_timestamps)
                errorTimeMax = data_OT_timestamps(error_index_minusone+1);
            else
                errorTimeMax = errorTimeStamp;
            end
        else
            errorTimeMin = errorTimeStamp;
            errorTimeMax = errorTimeStamp;
        end
        
        posMin = floor((errorTimeMin - startTime) / stepsize);
        posMax = ceil((errorTimeMax - startTime) / stepsize);
        for s=posMin:posMax
            if(s<=size(data_OT_common,1) && s > 0)
                %copy position before the error to erroneous locations
                data_OT_common{s} = data_OT_common{s-1};
                data_OT_common{s}.valid = 0;
            end
        end
    end
end

data_EMT_interpolated = synthetic_timestamps(data_EMT, [startTime endTime], frequencyHz, 'cpp');


% I store all timestamps in an array so i can compare to the error
% timestamps and find the predecessor and the successor. Not the finest
% solution but a working one.
numSen = size(data_EMT,2);
data_EMT_structarray_cell = cell(1,numSen);
data_EMT_timestamps_cell = cell(1,numSen);
for j=1:numSen
    data_EMT_structarray_cell{j} = [data_EMT{:,j}];
    data_EMT_timestamps_cell{j} = [data_EMT_structarray_cell{j}.TimeStamp];
end


% find entries that should be discarded because of missing sensor data
for j=1:numSen
    last_nonzero_index = find(data_EMT_timestamps_cell{j}~=0,1,'last');
    if size(errorTimeStampsEM, 2) >= j %maybe not all sensors had errors. If j bigger than size(errorTimeStampsEM, 2), size(errorTimeStampsEM(:,j) would give out an matlabError.
    for i = 1:size(errorTimeStampsEM(:,j),1)
        if ~isempty(errorTimeStampsEM{i,j})
            errorTimeStamp = errorTimeStampsEM{i,j};

            error_index_minusone = find(errorTimeStamp > data_EMT_timestamps_cell{j},1,'last');
            if ~isempty(error_index_minusone)
                errorTimeMin = data_EMT_timestamps_cell{j}(error_index_minusone);
                if error_index_minusone ~= last_nonzero_index
                    errorTimeMax = data_EMT_timestamps_cell{j}(error_index_minusone+1);
                else
                    errorTimeMax = errorTimeStamp;
                end
            else
                errorTimeMin = errorTimeStamp;
                errorTimeMax = errorTimeStamp;
            end
            
            posMin = floor((errorTimeMin - startTime) / stepsize);
            posMax = ceil((errorTimeMax - startTime) / stepsize);

            for s=posMin:posMax
                if(s<=size(data_EMT_interpolated,1) && s > 0)
                    data_EMT_interpolated{s,j} = data_EMT_interpolated{s-1,j}; %copy position before the error to erroneous locations
                    data_EMT_interpolated{s,j}.valid = 0;
                end
            end
        end
    end
    end
end

% create 4x4xN matrix for each Sensor, store them in a cell
[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT_interpolated, 'CppCodeQuat');


[H_commonEMT_to_EMCS, H_EMCS_to_commonEMT, data_EM_common] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity);
numPts = size(H_commonEMT_to_EMCS,3);
for i=1:numPts
    data_EM_common{i}.TimeStamp = data_EMT_interpolated{i,1}.TimeStamp;
end

%% use Fu,Fv and Fw (distortion map) to compute better EM positions
load('Fu_sd2');
load('Fv_sd2');
load('Fw_sd2');
amountNan = 0;
for i = 1:numPts
    if data_EM_common{i}.valid
       oldX = data_EM_common{i}.position(1);
       oldY = data_EM_common{i}.position(2);
       oldZ = data_EM_common{i}.position(3);
       if ~isnan(Fu(oldX, oldY, oldZ))
           data_EM_common{i}.position(1) = data_EM_common{i}.position(1) - Fu(oldX, oldY, oldZ);
       else
           i                   
           amountNan = amountNan+1;
       end
       if ~isnan(Fv(oldX, oldY, oldZ))
           data_EM_common{i}.position(2) = data_EM_common{i}.position(2) - Fv(oldX, oldY, oldZ);
       else
           i                   
           amountNan = amountNan+1;
       end
       if ~isnan(Fw(oldX, oldY, oldZ))
           data_EM_common{i}.position(3) = data_EM_common{i}.position(3) - Fw(oldX, oldY, oldZ);
       else
           i                   
           amountNan = amountNan+1;
       end
    end
end
amountNan

end