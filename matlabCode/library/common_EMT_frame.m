%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to one EMT and one OT dataset,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT.
%calibration is done for Optical to the first EM tracker.
%%%%%%%%%%%%%
function [frame, invframe] = common_EMT_frame(path, testrow_name_EMT)
% data read in
% do preparation
clear variables globals;
close all;

if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(pwd));
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack_corrupt'];

end
if ~exist('testrow_name_EMT', 'var')
    testrow_name_EMT = 'hybridEMT';
end


% get data for hand/eye calib
[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);

% create 4x4xN matrix for each Sensor, store them in a cell
[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT);

if size(H_EMT_to_EMCS_cell, 2) > 1
    % plot position data
    figurehandle = Plot_frames(H_EMT_to_EMCS_cell(2:end));
    Plot_points(H_EMT_to_EMCS_cell(1), figurehandle);

    % get average EMT H_differences
    numPts = size(data_EMT,1);
    numSen = size(data_EMT,2);
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
    frame = zeros(4,4,numPts);
    frameWithoutError = zeros(4,4,1);
    errorPoints = 0;
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
        if (goodSens == 0) %in case no sensor is good: no new entry in frameWithoutError 
            errorPoints = errorPoints + 1;
        else
            frameWithoutError(:,:,i-errorPoints) = frame(:,:,i)/goodSens; %numSen;
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
    Plot_points(wrappercell, 2);
else
    frame = H_EMT_to_EMCS_cell{1};
    numPts = size(data_EMT,1);
    invframe = zeros(4,4,numPts);
    for i=1:numPts
        invframe(:,:,i) = inv(frame(:,:,i));
    end
end

end