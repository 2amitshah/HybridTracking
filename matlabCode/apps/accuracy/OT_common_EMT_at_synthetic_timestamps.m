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

function [H_commonEMT_to_EMCS, H_EMCS_to_commonEMT, data_EM_common, data_OT_common] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EM, testrow_name_OT, frequencyHz, verbosity)
% common_EMT_frame should be located in \library

% data read in
% do preparation

% close all;

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
[data_OT, data_EMT, errorTimeStampsOT, errorTimeStampsEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EM, 1);
% determine earliest and latest common timestamp
[interval] = obtain_boundaries_for_interpolation(data_OT, data_EMT);
startTime = interval(1);
endTime = interval(2);

%set up wished timestamps
stepsize = 1*10^9 / frequencyHz;

%% old code
% %the maximum distance in which we will use the measurements, else we assume that there had been an outlier
% maxDistance = stepsize; 
% 
% numSen = size(data_EMT,2);
% 
% for i = 1:size(data_EMT,1)
%     for j = 1:size(data_EMT,2)
%         if (~isempty(data_EMT{i,j}))
%             switch j;
%                 case 1
%                     allTimeStampsEMT1(i) = data_EMT{i,j}.TimeStamp;
%                 case 2
%                     allTimeStampsEMT2(i) = data_EMT{i,j}.TimeStamp;
%                 case 3
%                     allTimeStampsEMT3(i) = data_EMT{i,j}.TimeStamp;
%             end
%         end
%     end
% end
% for i = 1:size(data_EMT,1)
%     for j = 1:size(data_EMT,2)
%         if (~isempty(data_EMT{i,j}))
%             switch j;
%                 case 1
%                     allDataEMT1Pos1(i) = data_EMT{i,j}.position(1);
%                     allDataEMT1Pos2(i) = data_EMT{i,j}.position(2);
%                     allDataEMT1Pos3(i) = data_EMT{i,j}.position(3);
%                     allDataEMT1Or1(i) = data_EMT{i,j}.orientation(1);
%                     allDataEMT1Or2(i) = data_EMT{i,j}.orientation(2);
%                     allDataEMT1Or3(i) = data_EMT{i,j}.orientation(3);
%                     allDataEMT1Or4(i) = data_EMT{i,j}.orientation(4);                    
%                 case 2
%                     allDataEMT2Pos1(i) = data_EMT{i,j}.position(1);
%                     allDataEMT2Pos2(i) = data_EMT{i,j}.position(2);
%                     allDataEMT2Pos3(i) = data_EMT{i,j}.position(3);
%                     allDataEMT2Or1(i) = data_EMT{i,j}.orientation(1);
%                     allDataEMT2Or2(i) = data_EMT{i,j}.orientation(2);
%                     allDataEMT2Or3(i) = data_EMT{i,j}.orientation(3);
%                     allDataEMT2Or4(i) = data_EMT{i,j}.orientation(4);
%                 case 3
%                     allDataEMT3Pos1(i) = data_EMT{i,j}.position(1);
%                     allDataEMT3Pos2(i) = data_EMT{i,j}.position(2);
%                     allDataEMT3Pos3(i) = data_EMT{i,j}.position(3);
%                     allDataEMT3Or1(i) = data_EMT{i,j}.orientation(1);
%                     allDataEMT3Or2(i) = data_EMT{i,j}.orientation(2);
%                     allDataEMT3Or3(i) = data_EMT{i,j}.orientation(3);
%                     allDataEMT3Or4(i) = data_EMT{i,j}.orientation(4);
%             end
%         end
%     end
% end
% for j = 1:size(data_EMT,2)
%     switch j;
%         case 1
%             [allTimeStampsEMT1_withoutrep,indicesEMT1_withoutrep,~] = unique(allTimeStampsEMT1);
%         case 2
%             [allTimeStampsEMT2_withoutrep,indicesEMT2_withoutrep,~] = unique(allTimeStampsEMT2);
%         case 3
%             [allTimeStampsEMT3_withoutrep,indicesEMT3_withoutrep,~] = unique(allTimeStampsEMT3);
%     end
% end
% 
% 
% %why numSen+1? QUEST, RESOLVED: numSen is number of EM sensors, plus 1
% %because of the optical sensor
% measurements_syntheticTimeStamps = cell(floor((endTime - startTime) / stepsize) + 1, numSen+1);
%% end old code

% create data_OT_common as an interpolation at defined timestamps, all
% entrys are marked as .valid = 1 as default
data_OT_common = synthetic_timestamps(data_OT, [startTime endTime], frequencyHz, 'cpp');

% SOLUTION1
% I store all timestamps in an array so i can compare to the error
% timestamps and find the predecessor and the successor. Not the finest
% solution but a working one.
data_OT_structarray = [data_OT{:}];
data_OT_timestamps = [data_OT_structarray.TimeStamp];
% end SOLUTION1

% find entries that should be discarded because of missing sensor data
for i = 1:size(errorTimeStampsOT,1)
    if ~isempty(errorTimeStampsOT{i}) %how could that happen? QUEST
        errorTimeStamp = errorTimeStampsOT{i}; %why were they stored in a cell in the first place? QUEST
        %--------
        % DEBUG1 so here posMin and posMax have to be redefined:
        % posMin has to be one stepsize before the timestamp that we had
        % BEFORE the erroneous one. Equally, posMax has to be one stepsize
        % after the timestamp AFTER the erroneous one.
        %--------
%         errorTimeMin = errorTimeStamp; % - stepsize;
%         errorTimeMax = errorTimeStamp; % + stepsize;
        %compute corresponding positions in %measurements_syntheticTimeStamps
%         posMin = floor((errorTimeMin - startTime) / stepsize);
%         posMax = ceil((errorTimeMax - startTime) / stepsize);
        %--------
        % end DEBUG1
        %--------
        
        % SOLUTION1
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
        % end SOLUTION1
        for s=posMin:posMax
            if(s<=size(data_OT_common,1) && s > 0)
                data_OT_common{s} = data_OT_common{s-1}; %copy position before the error to erroneous locations
                data_OT_common{s}.valid = 0;
            end
        end
    end
end

%% old code
% s = 1;
% for t = startTime:stepsize:endTime
%     valOTt.position(1) = 0;
%     valOTt.position(2) = 0;
%     valOTt.position(3) = 0;
%     valOTt.orientation(1) = 0;
%     valOTt.orientation(2) = 0;
%     valOTt.orientation(3) = 0;
%     valOTt.orientation(4) = 0;%should come from santiago
%     measurements_syntheticTimeStamps{s,1} = valOTt; % OT value at timestamp t
%     for i = 1:numSen
%         switch i;
%             case 1
%                 val.position(1) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Pos1(1,indicesEMT1_withoutrep), t);
%                 val.position(2) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Pos2(1,indicesEMT1_withoutrep), t);
%                 val.position(3) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Pos3(1,indicesEMT1_withoutrep), t);
%                 val.orientation(1) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or1(1,indicesEMT1_withoutrep), t);
%                 val.orientation(2) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or2(1,indicesEMT1_withoutrep), t);
%                 val.orientation(3) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or3(1,indicesEMT1_withoutrep), t);
%                 val.orientation(4) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or4(1,indicesEMT1_withoutrep), t);
%             case 2
%                 val.position(1) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Pos1(1,indicesEMT2_withoutrep), t);
%                 val.position(2) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Pos2(1,indicesEMT2_withoutrep), t);
%                 val.position(3) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Pos3(1,indicesEMT2_withoutrep), t);
%                 val.orientation(1) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or1(1,indicesEMT2_withoutrep), t);
%                 val.orientation(2) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or2(1,indicesEMT2_withoutrep), t);
%                 val.orientation(3) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or3(1,indicesEMT2_withoutrep), t);
%                 val.orientation(4) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or4(1,indicesEMT2_withoutrep), t);
%             case 3
%                 val.position(1) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Pos1(1,indicesEMT3_withoutrep), t);
%                 val.position(2) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Pos2(1,indicesEMT3_withoutrep), t);
%                 val.position(3) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Pos3(1,indicesEMT3_withoutrep), t);
%                 val.orientation(1) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or1(1,indicesEMT3_withoutrep), t);
%                 val.orientation(2) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or2(1,indicesEMT3_withoutrep), t);
%                 val.orientation(3) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or3(1,indicesEMT3_withoutrep), t);
%                 val.orientation(4) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or4(1,indicesEMT3_withoutrep), t);
%         end
%         measurements_syntheticTimeStamps{s, i+1} = val; % EM value of sensor i at timestamp t       
%     end   
%     s = s+1;
% end
%% end of old code
data_EMT_interpolated = synthetic_timestamps(data_EMT, [startTime endTime], frequencyHz, 'cpp');

% SOLUTION1
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
% end SOLUTION1

% find entries that should be discarded because of missing sensor data
for j=1:numSen
    last_nonzero_index = find(data_EMT_timestamps_cell{j}~=0,1,'last');
    if size(errorTimeStampsEM, 2) >= j %maybe not all sensors had errors. If j bigger than size(errorTimeStampsEM, 2), size(errorTimeStampsEM(:,j) would give out an matlabError.
    for i = 1:size(errorTimeStampsEM(:,j),1)
        if ~isempty(errorTimeStampsEM{i,j}) %how could that happen? QUEST

            %why were they stored in a cell in the first place? QUEST,
            %RESOLVED: because for EM data a cell is necessary. So OT data
            %is prepared for also having multiple sensors in the future.
            %also the two outputs are consistent, being both cells.
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
            % end SOLUTION1
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

end