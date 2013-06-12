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

function [frame, invframe, data_EM_common, data_OT_common] = OT_and_common_EMT_at_synthetic_timestamps_V2(path, testrow_name_EM, testrow_name_OT)
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


measurements_syntheticTimeStamps = cell(((endTime - startTime) / stepsize) + 1, numSen+1);
data_OT_common = synthetic_timestamps(data_OT, [startTime endTime], frequencyHz);
data_EM_interpolated = synthetic_timestamps(data_EMT, [startTime endTime], frequencyHz);



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
                    data_EM_interpolated{s,j}.valid = 0;
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
        posMax = ceil((errorTimeMax - startTime) / stepsize);
        for s=posMin:posMax
            data_OT_common{s}.valid = 0;
        end
    end
end


% create 4x4xN matrix for each Sensor, store them in a cell
[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EM_interpolated, 'CppCodeQuat');


if numSen > 1
    % plot position data
    figurehandle = figure;
    Plot_points(H_EMT_to_EMCS_cell(2:end),figurehandle, 2);
    Plot_points(H_EMT_to_EMCS_cell(1), figurehandle, 1);

    % get average EMT H_differences
    numPts = size(data_EM_interpolated,1);
    
    
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
            if ~(data_EM_interpolated{i,j}.valid && data_EM_interpolated{i,1}.valid)
                errorPoints = errorPoints+1;
            else    
                H_diff{j-1}(:,:,i-errorPoints) = inv(H_EMT_to_EMCS_cell{1}(:,:,i))*H_EMT_to_EMCS_cell{j}(:,:,i);
            end
        end
        H_diff{j-1}(:,:,1) = mean(H_diff{j-1}(:,:,:),3); 
        H_diff{j-1} = H_diff{j-1}(:,:,1); %H_diff contains only one transformation matrix
        % normalize to have a correct rotation matrix
        for col = 1:3
            H_diff{j-1}(1:3,col) = H_diff{j-1}(1:3,col) / norm(H_diff{j-1}(1:3,col)); 
        end
    end

    % project every EMT 2 etc to EMT 1, build average
    data_EM_common = cell(1,1);
    frame = zeros(4,4,numPts);
    errorPoints = 0;
    check_available_EMT_sensors_figure = figure;
    for i=1:numPts
        goodSens = 0;
        if data_EM_interpolated{i,1}.valid            
            frame(:,:,i) = H_EMT_to_EMCS_cell{1}(:,:,i);
            goodSens = goodSens + 1;
        end
        
        for j=2:numSen
            if data_EM_interpolated{i,j}.valid
                H_new{j-1} = H_EMT_to_EMCS_cell{j}(:,:,i)*inv(H_diff{j-1});
                frame(:,:,i) = frame(:,:,i) + H_new{j-1};
                goodSens = goodSens + 1;
            end
        end
                  
            
        % plot number of available sensors
        hold on
        plot(i,goodSens, 'x');
        hold off
        data_EM_common{i,1}.TimeStamp = startTime + (i-1) * stepsize;
        if (goodSens == 0) %in case no sensor is good: no new entry in frameWithoutError,
                           %same entry again in data_EM_common..?
            errorPoints = errorPoints + 1;
            data_EM_common{i,1}.position = [0 0 0];
            data_EM_common{i,1}.orientation(1:4) = [0 0 0 0];
        else
            frame(:,:,i) = frame(:,:,i)/goodSens; %numSen;
            data_EM_common{i,1}.position(1:3) = frame(1:3,4,i)';
            for col = 1:3
                frame(1:3,col,i) = frame(1:3,col,i) / norm(frame(1:3,col,i)); 
            end
            R = frame(1:3, 1:3, i);
            data_EM_common{i,1}.orientation(1:4) = rot2quat_q41(R);
        end
    end

    invframe = zeros(4,4,size(frame,3));
    for i=1:size(frame,3)
        invframe(:,:,i) = inv(frame(:,:,i));
    end
    wrappercell{1}=frame;

    % plot position data of synthesized position
    Plot_frames(wrappercell, figurehandle, 5);
    
else
    frame = H_EMT_to_EMCS_cell{1};
    numPts = size(data_EM_interpolated,1);
    invframe = zeros(4,4,numPts);
    for i=1:numPts
        invframe(:,:,i) = inv(frame(:,:,i));
    end
end

end