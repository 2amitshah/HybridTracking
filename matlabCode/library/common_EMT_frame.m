%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to one EMT and one OT dataset,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT.
%calibration is done for Optical to the first EM tracker.
%%%%%%%%%%%%%
function [frame, invframe] = common_EMT_frame(path, testrow_name_EMT)
% common_EMT_frame should be located in \library

% data read in
% do preparation
close all;

if ~exist('path', 'var')
    % common_emt_frame is in \library\
    pathGeneral = fileparts(fileparts(fileparts(which(mfilename))));
    path = [pathGeneral filesep 'measurements' filesep '05.29 Measurements'];
end
if ~exist('testrow_name_EMT', 'var')
    testrow_name_EMT = 'hybridEMT';
end

% get data for hand/eye calib
[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);

% create 4x4xN matrix for each Sensor, store them in a cell
[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT, 'NDIQuat');

numPts = size(data_EMT,1);
numSen = size(data_EMT,2);
if numSen > 1
    % plot position data
    figurehandle = Plot_points(H_EMT_to_EMCS_cell(2:end),[], 2);
    Plot_points(H_EMT_to_EMCS_cell(1), figurehandle, 1); %EMT1 is blue

    % get average EMT H_differences
    H_diff=cell(1,numSen-1);

    for j=2:numSen
        errorPoints = 0;
        for i=1:numPts
            %calculate position of sensors 2, 3, etc relative to sensor 1
            %check translations in these matrices.. if any of both is
            %bad: don't add to H_diff
%             H_EMT_to_EMCS_cell{j}(1,4,i)
%             H_EMT_to_EMCS_cell{j}(2,4,i)
%             H_EMT_to_EMCS_cell{j}(3,4,i)
            if (abs(H_EMT_to_EMCS_cell{1}(1,4,i)) > 10000 || abs(H_EMT_to_EMCS_cell{j}(1,4,i)) > 10000)
                errorPoints = errorPoints+1;
            else
                % H_diff points from the sensor 2, 3, ... to sensor 1
                H_diff{j-1}(:,:,i-errorPoints) = inv(H_EMT_to_EMCS_cell{1}(:,:,i))*H_EMT_to_EMCS_cell{j}(:,:,i);
            end
        end
        H_diff{j-1}(:,:,1) = mean(H_diff{j-1}(:,:,:),3); 
        H_diff{j-1} = H_diff{j-1}(:,:,1); %H_diff contains only one transformation matrix
        for col = 1:3
            % normalize rotation matrix vectors
            H_diff{j-1}(1:3,col)=H_diff{j-1}(1:3,col)/norm(H_diff{j-1}(1:3,col));
        end
    end

    % project every EMT 2 etc to EMT 1, build average
    frame = zeros(4,4,numPts);
    frameWithoutError = zeros(4,4,1);
    H_new = cell(1,numSen-1);
    errorPoints = 0;
    for i=1:numPts
        goodSens = 0;
        if ( abs(H_EMT_to_EMCS_cell{1}(1,4,i)) > 10000 )
            % do nothing
        else            
            frame(:,:,i) = H_EMT_to_EMCS_cell{1}(:,:,i);
            goodSens = goodSens + 1;
        end
        for j=2:numSen
            if ( abs(H_EMT_to_EMCS_cell{j}(1,4,i)) > 10000 )
                % do nothing
            else
                % from sensor 1 to sensor 2, 3, ... to origin, so basically
                % from sensor 1 to origin. can be added to EMT to EMCS.
                H_new{j-1} = H_EMT_to_EMCS_cell{j}(:,:,i)*inv(H_diff{j-1});
                frame(:,:,i) = frame(:,:,i) + H_new{j-1};
                goodSens = goodSens + 1;
            end
        end
        % very ugly mean value creation        
        if (goodSens == 0) %in case no sensor is good: no new entry in frameWithoutError 
            errorPoints = errorPoints + 1;
        else
            frameWithoutError(:,:,i-errorPoints) = frame(:,:,i)/goodSens; %numSen;
            for col = 1:3
                % normalize rotation matrix vectors
                frameWithoutError(1:3,col,i-errorPoints)=frameWithoutError(1:3,col,i-errorPoints)/norm(frameWithoutError(1:3,col,i-errorPoints));
            end
        end
    end
    frame = frameWithoutError;
    wrappercell{1}=frame;

    % plot position data of synthesized position
    Plot_points(wrappercell, figurehandle, 5);
else
    frame = H_EMT_to_EMCS_cell{1};
end
invframe = zeros(4,4,size(frame,3));
for i=1:size(frame,3)
    invframe(:,:,i) = inv(frame(:,:,i));
end

end