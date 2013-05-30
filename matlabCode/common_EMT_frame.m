%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to one EMT and one OT dataset,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT.
%calibration is done for Optical to the first EM tracker.
%%%%%%%%%%%%%
function [frame] = common_EMT_frame(path, testrow_name_EMT)
% data read in
% do preparation
clear variables globals;
close all;

if ~exist('path', 'var')
    path = '..\measurements\testmfrom_NDItrack';
end
if ~exist('testrow_name_EMT', 'var')
    testrow_name_EMT = 'hybridEMT';
end


% get data for hand/eye calib
[data_EMT, ~, ~] = read_NDI_tracking_files(path, testrow_name_EMT);

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
        for i=1:numPts
            %calculate position of sensors 2, 3, etc relative to sensor 1
            H_diff{j-1}(:,:,i) = inv(H_EMT_to_EMCS_cell{1}(:,:,i))*H_EMT_to_EMCS_cell{j}(:,:,i);
        end
        H_diff{j-1}(:,:,1) = mean(H_diff{j-1}(:,:,:),3);
        H_diff{j-1} = H_diff{j-1}(:,:,1);
    end

    % project every EMT 2 etc to EMT 1, build average
    frame = zeros(4,4,numPts);
    for i=1:numPts
        frame(:,:,i) = H_EMT_to_EMCS_cell{1}(:,:,i);
        for j=2:numSen
            H_new{j-1} = H_EMT_to_EMCS_cell{j}(:,:,i)*inv(H_diff{j-1});
            frame(:,:,i) = frame(:,:,i) + H_new{j-1};
        end
        % very ugly mean value creation
        frame(:,:,i) = frame(:,:,i)/numSen;
    end

    wrappercell = cell(1);
    wrappercell{1}=frame;

    % plot position data of synthesized position
    Plot_points(wrappercell, figurehandle);
else
    frame = H_EMT_to_EMCS_cell{1};
end

end