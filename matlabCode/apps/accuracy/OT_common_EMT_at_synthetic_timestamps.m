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
%
%OUTPUT
%frame: contains matrices with the computed EM data of one sensor
%
%invframe: contains the inverse matrices of the frame
%
%data_EM_common: contains the computed EM data at the position of EM sensor
%                1 at the synthetic timestamps
%
%data_OT_common: contains the computed OT data at the synthetic timestamps
%
%%%%%%%%%%%%%

function [frame, invframe, data_EM_common, data_OT_common] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EM, testrow_name_OT)
% common_EMT_frame should be located in \library

% data read in
% do preparation

close all;

frequencyHz = 40;

if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];

end
if ~exist('testrow_name_EM', 'var')
    testrow_name_EM = 'EMTracking_cont';
end
if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OpticalTracking_cont';
end

% get data for hand/eye calib
%[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
[data_OT, data_EMT, errorTimeStampsOT, errorTimeStampsEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EM, 1);
startTime = data_OT{1,1}.TimeStamp;
if data_EMT{1,1}.TimeStamp > startTime
    startTime = data_EMT{1,1}.TimeStamp;
end

endTime = data_OT{size(data_OT,1),1}.TimeStamp;
sizeEMT = size(data_EMT,1);
maxIndex = 1;
for i = 1:size(data_EMT,2)
    if (~isempty(data_EMT{sizeEMT,i}))
        maxIndex = i;
    end
end
if data_EMT{sizeEMT,maxIndex}.TimeStamp < endTime
    endTime = data_EMT{sizeEMT,maxIndex}.TimeStamp;
end

%set up wished timestamps
stepsize = 1*10^9 / frequencyHz;
%the maximum distance in which we will use the measurements, else we assume that there had been an outlier
maxDistance = stepsize; 

numSen = size(data_EMT,2);

for i = 1:size(data_EMT,1)
    for j = 1:size(data_EMT,2)
        if (~isempty(data_EMT{i,j}))
            switch j;
                case 1
                    allTimeStampsEMT1(i) = data_EMT{i,j}.TimeStamp;
                case 2
                    allTimeStampsEMT2(i) = data_EMT{i,j}.TimeStamp;
                case 3
                    allTimeStampsEMT3(i) = data_EMT{i,j}.TimeStamp;
            end
        end
    end
end
for i = 1:size(data_EMT,1)
    for j = 1:size(data_EMT,2)
        if (~isempty(data_EMT{i,j}))
            switch j;
                case 1
                    allDataEMT1Pos1(i) = data_EMT{i,j}.position(1);
                    allDataEMT1Pos2(i) = data_EMT{i,j}.position(2);
                    allDataEMT1Pos3(i) = data_EMT{i,j}.position(3);
                    allDataEMT1Or1(i) = data_EMT{i,j}.orientation(1);
                    allDataEMT1Or2(i) = data_EMT{i,j}.orientation(2);
                    allDataEMT1Or3(i) = data_EMT{i,j}.orientation(3);
                    allDataEMT1Or4(i) = data_EMT{i,j}.orientation(4);                    
                case 2
                    allDataEMT2Pos1(i) = data_EMT{i,j}.position(1);
                    allDataEMT2Pos2(i) = data_EMT{i,j}.position(2);
                    allDataEMT2Pos3(i) = data_EMT{i,j}.position(3);
                    allDataEMT2Or1(i) = data_EMT{i,j}.orientation(1);
                    allDataEMT2Or2(i) = data_EMT{i,j}.orientation(2);
                    allDataEMT2Or3(i) = data_EMT{i,j}.orientation(3);
                    allDataEMT2Or4(i) = data_EMT{i,j}.orientation(4);
                case 3
                    allDataEMT3Pos1(i) = data_EMT{i,j}.position(1);
                    allDataEMT3Pos2(i) = data_EMT{i,j}.position(2);
                    allDataEMT3Pos3(i) = data_EMT{i,j}.position(3);
                    allDataEMT3Or1(i) = data_EMT{i,j}.orientation(1);
                    allDataEMT3Or2(i) = data_EMT{i,j}.orientation(2);
                    allDataEMT3Or3(i) = data_EMT{i,j}.orientation(3);
                    allDataEMT3Or4(i) = data_EMT{i,j}.orientation(4);
            end
        end
    end
end
for j = 1:size(data_EMT,2)
    switch j;
        case 1
            [allTimeStampsEMT1_withoutrep,indicesEMT1_withoutrep,~] = unique(allTimeStampsEMT1);
        case 2
            [allTimeStampsEMT2_withoutrep,indicesEMT2_withoutrep,~] = unique(allTimeStampsEMT2);
        case 3
            [allTimeStampsEMT3_withoutrep,indicesEMT3_withoutrep,~] = unique(allTimeStampsEMT3);
    end
end

s = 1;
measurements_syntheticTimeStamps = cell(((endTime - startTime) / stepsize) + 1, numSen+1);
data_OT_common = synthetic_timestamps(data_OT, [startTime endTime], frequencyHz);

for t = startTime:stepsize:endTime
    valOTt.position(1) = 0;
    valOTt.position(2) = 0;
    valOTt.position(3) = 0;
    valOTt.orientation(1) = 0;
    valOTt.orientation(2) = 0;
    valOTt.orientation(3) = 0;
    valOTt.orientation(4) = 0;%should come from santiago
    measurements_syntheticTimeStamps{s,1} = valOTt; % OT value at timestamp t
    for i = 1:numSen
        switch i;
            case 1
                val.position(1) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Pos1(1,indicesEMT1_withoutrep), t);
                val.position(2) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Pos2(1,indicesEMT1_withoutrep), t);
                val.position(3) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Pos3(1,indicesEMT1_withoutrep), t);
                val.orientation(1) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or1(1,indicesEMT1_withoutrep), t);
                val.orientation(2) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or2(1,indicesEMT1_withoutrep), t);
                val.orientation(3) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or3(1,indicesEMT1_withoutrep), t);
                val.orientation(4) = interp1(allTimeStampsEMT1_withoutrep, allDataEMT1Or4(1,indicesEMT1_withoutrep), t);
            case 2
                val.position(1) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Pos1(1,indicesEMT2_withoutrep), t);
                val.position(2) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Pos2(1,indicesEMT2_withoutrep), t);
                val.position(3) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Pos3(1,indicesEMT2_withoutrep), t);
                val.orientation(1) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or1(1,indicesEMT2_withoutrep), t);
                val.orientation(2) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or2(1,indicesEMT2_withoutrep), t);
                val.orientation(3) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or3(1,indicesEMT2_withoutrep), t);
                val.orientation(4) = interp1(allTimeStampsEMT2_withoutrep, allDataEMT2Or4(1,indicesEMT2_withoutrep), t);
            case 3
                val.position(1) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Pos1(1,indicesEMT3_withoutrep), t);
                val.position(2) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Pos2(1,indicesEMT3_withoutrep), t);
                val.position(3) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Pos3(1,indicesEMT3_withoutrep), t);
                val.orientation(1) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or1(1,indicesEMT3_withoutrep), t);
                val.orientation(2) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or2(1,indicesEMT3_withoutrep), t);
                val.orientation(3) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or3(1,indicesEMT3_withoutrep), t);
                val.orientation(4) = interp1(allTimeStampsEMT3_withoutrep, allDataEMT3Or4(1,indicesEMT3_withoutrep), t);
        end
        measurements_syntheticTimeStamps{s, i+1} = val; % EM value of sensor i at timestamp t       
    end   
    s = s+1;
end

%get rid of those values at whose timestamps we've had an error as the
%interpolation could be far from the real position in space
for i = 1:size(errorTimeStampsEM,1)
    for j = 1:size(errorTimeStampsEM,2) %amount of sensors
        if (~isempty(errorTimeStampsEM{i,j}))
            if (errorTimeStampsEM{i,j}~=0)
                errorTimeStamp = errorTimeStampsEM{i,j};
                errorTimeMin = errorTimeStamp; % - stepsize;
                errorTimeMax = errorTimeStamp; % + stepsize;
                %compute corresponding positions in %measurements_syntheticTimeStamps
                posMin = floor((errorTimeMin - startTime) / stepsize);
                posMax = ceil((errorTimeMax - startTime) / stepsize);
                for s=posMin:posMax
                    for x = 1:3
                        measurements_syntheticTimeStamps{s,j+1}.position(x) = -100000; 
                    end
                end
            end
        end
    end
end

for i = 1:size(errorTimeStampsOT,1)
    if (errorTimeStampsOT{i}~=0)
        errorTimeStamp = errorTimeStampsOT{i};
        errorTimeMin = errorTimeStamp; % - stepsize;
        errorTimeMax = errorTimeStamp; % + stepsize;
        %compute corresponding positions in %measurements_syntheticTimeStamps
        posMin = floor((errorTimeMin - startTime) / stepsize);
        posMax = ceil((errorTimeMax - startTime) / stepsize)
        for s=posMin:posMax
            for x = 1:3
                measurements_syntheticTimeStamps{s,1}.position(x) = -100000; 
            end
        end
    end
end


% create 4x4xN matrix for each Sensor, store them in a cell
%[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(measurements_syntheticTimeStamps(:,2:size(measurements_syntheticTimeStamps,2)), 'CppCodeQuat');


if size(H_EMT_to_EMCS_cell, 2) > 1
    % plot position data
    figurehandle = Plot_frames(H_EMT_to_EMCS_cell(2:end));
    Plot_points(H_EMT_to_EMCS_cell(1), figurehandle);

    % get average EMT H_differences
    %numPts = size(data_EMT,1);
    numPts = size(measurements_syntheticTimeStamps,1);
    
    H_diff=cell(1,numSen-1);

    for j=2:numSen
        errorPoints = 0;
        for i=1:numPts
            %calculate position of sensors 2, 3, etc relative to sensor 1
            %check translations in these matrices.. if any of both is
            %bad: don't add to H_diff
            %check if a point exists for the wished timestamp
%             H_EMT_to_EMCS_cell{j}(1,4,i)
%             H_EMT_to_EMCS_cell{j}(2,4,i)
%             H_EMT_to_EMCS_cell{j}(3,4,i)
            if (H_EMT_to_EMCS_cell{1}(1,4,i) < -10000 || H_EMT_to_EMCS_cell{1}(1,4,i) > 10000 || ...
                H_EMT_to_EMCS_cell{j}(1,4,i) < -10000 || H_EMT_to_EMCS_cell{j}(1,4,i) > 10000 || ...
                H_EMT_to_EMCS_cell{1}(2,4,i) < -10000 || H_EMT_to_EMCS_cell{1}(2,4,i) > 10000 || ...
                H_EMT_to_EMCS_cell{j}(2,4,i) < -10000 || H_EMT_to_EMCS_cell{j}(2,4,i) > 10000 || ...
                H_EMT_to_EMCS_cell{1}(3,4,i) < -10000 || H_EMT_to_EMCS_cell{1}(3,4,i) > 10000 || ...
                H_EMT_to_EMCS_cell{j}(3,4,i) < -10000 || H_EMT_to_EMCS_cell{j}(3,4,i) > 10000 )
            
                errorPoints = errorPoints+1;
            else    
                H_diff{j-1}(:,:,i-errorPoints) = inv(H_EMT_to_EMCS_cell{1}(:,:,i))*H_EMT_to_EMCS_cell{j}(:,:,i);
            end
        end
        H_diff{j-1}(:,:,1) = mean(H_diff{j-1}(:,:,:),3); 
        H_diff{j-1} = H_diff{j-1}(:,:,1); %H_diff contains only one transformation matrix
    end

    % project every EMT 2 etc to EMT 1, build average
    data_EM_common = cell(1,1);
    frame = zeros(4,4,numPts);
    frameWithoutError = zeros(4,4,1);
    errorPoints = 0;
    figure
    for i=1:numPts
        goodSens = 0;
        if ( H_EMT_to_EMCS_cell{1}(1,4,i) < -10000 || H_EMT_to_EMCS_cell{1}(1,4,i) > 10000 || ...
             H_EMT_to_EMCS_cell{1}(2,4,i) < -10000 || H_EMT_to_EMCS_cell{1}(2,4,i) > 10000 || ...
             H_EMT_to_EMCS_cell{1}(3,4,i) < -10000 || H_EMT_to_EMCS_cell{1}(3,4,i) > 10000 )
        else            
            frame(:,:,i) = H_EMT_to_EMCS_cell{1}(:,:,i);
            goodSens = goodSens + 1;
        end
        
        for j=2:numSen
            if ( H_EMT_to_EMCS_cell{j}(1,4,i) < -10000 || H_EMT_to_EMCS_cell{j}(1,4,i) > 10000 || ...
                 H_EMT_to_EMCS_cell{j}(2,4,i) < -10000 || H_EMT_to_EMCS_cell{j}(2,4,i) > 10000 || ...
                 H_EMT_to_EMCS_cell{j}(3,4,i) < -10000 || H_EMT_to_EMCS_cell{j}(3,4,i) > 10000 )
            else            
                H_new{j-1} = H_EMT_to_EMCS_cell{j}(:,:,i)*inv(H_diff{j-1});
                frame(:,:,i) = frame(:,:,i) + H_new{j-1};
                goodSens = goodSens + 1;
            end
        end
        % very ugly mean value creation     
        plot(i,goodSens, 'x');
        hold on
        data_EM_common{i,1}.TimeStamp = startTime + i* stepsize;
        if (goodSens == 0) %in case no sensor is good: no new entry in frameWithoutError,
                           %same entry again in data_EM_common..?
            errorPoints = errorPoints + 1;
            data_EM_common{i,1}.position = data_EM_common{i-1,1}.position;
            data_EM_common{i,1}.orientation(1:4) = data_EM_common{i-11}.orientation;
        else
            frameWithoutError(:,:,i-errorPoints) = frame(:,:,i)/goodSens; %numSen;
            %data_EM_common{i-errorPoints,1}.TimeStamp = startTime + i * stepsize;
            %data_EM_common{i-errorPoints,1}.position(1:3) = frame(1:3,4,i) / frame(4,4,i);
            data_EM_common{i,1}.position(1:3) = frame(1:3,4,i) / frame(4,4,i);
            R = frame(1:3, 1:3, i);
            R = R./frame(4,4,i);
            %data_EM_common{i-errorPoints,1}.orientation(1:4) = rot2quat_q41(R);
            data_EM_common{i,1}.orientation(1:4) = rot2quat_q41(R);
            %invframe(:,:,i-errorPoints) = inv(finalframe(:,:,i-errorPoints));
        end
    end

    frame = frameWithoutError;
    invframe = zeros(4,4,size(frame,3));
    for i=1:size(frame,3)
        invframe(:,:,i) = inv(frame(:,:,i));
    end
    wrappercell{1}=frame;

    % plot position data of synthesized position
    %Plot_points(wrappercell, figurehandle);
    wrappercellOT{1} = measurements_syntheticTimeStamps(:,1);
    Plot_points(wrappercell, 3);    
    
else
    frame = H_EMT_to_EMCS_cell{1};
    numPts = size(data_EMT,1);
    invframe = zeros(4,4,numPts);
    for i=1:numPts
        invframe(:,:,i) = inv(frame(:,:,i));
    end
end

end