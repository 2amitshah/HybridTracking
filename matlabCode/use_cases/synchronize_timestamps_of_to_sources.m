%% USE CASE
% Tracking data read-in. Then a point-could-fit is used to guess the
% timestamp-offset.
% Author: Felix Achilles, July 2013

close all; clear all;

currentPath = which('read_raw_data_from_TrackingFusion.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.30_Measurements'];
testrow_name_EMT = 'EMTrackingAscensionDirect';
testrow_name_OT = 'OpticalTrackingDirect';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

[dataOT, dataEM] = read_Direct_OpticalAndAscension(filenames_struct, [], [], 'vRelease');

[H_EMT_to_EMCS_sOne_interp] = frame_interpolation(dataEM(:,1), [], 20, 'ndi');
[H_OT_to_OCS_interp] = frame_interpolation(dataOT(:,1), [], 20, 'cpp');

size(H_EMT_to_EMCS_sOne_interp, 3);
size(H_OT_to_OCS_interp, 3);

% create a weights vector
[minimal_num_pts, minIndx] = min([size(H_EMT_to_EMCS_sOne_interp, 3), size(H_OT_to_OCS_interp, 3)]);
weightsVector = zeros(minimal_num_pts,1);
if minIndx == 1
    tmpFrames = H_EMT_to_EMCS_sOne_interp;
elseif minIdx == 2
    tmpFrames = H_OT_to_OCS_interp;
end
for j = 1:minimal_num_pts-1
    weightsVector(j) = norm(tmpFrames(1:3,4,j+1) - tmpFrames(1:3,4,j));
end
weightsVector(weightsVector > 10) = 0;
weightsVector = weightsVector.^2;

% feature: use generic weighting function that outputs the speed in mm/s
% and 0 if the movement was slower than 5 mm/s. To not overweight with
% speed, it is blocked at 100 mm/s
% weightsVector = weightFcn(1:minimal_num_pts,tmpFrames);
% works okay, at least with the icp afterwards

% fit the last points of the recordings to one another
if size(H_EMT_to_EMCS_sOne_interp, 3) > size(H_OT_to_OCS_interp, 3)
    T = absor(permute(H_OT_to_OCS_interp(1:3,4,:),[1 3 2]), permute(H_EMT_to_EMCS_sOne_interp(1:3,4,(end-size(H_OT_to_OCS_interp, 3)+1):end),[1 3 2]), 'weights', weightsVector);
else
    T = absor(permute(H_OT_to_OCS_interp(1:3,4,(end-size(H_EMT_to_EMCS_sOne_interp, 3)+1):end),[1 3 2]), permute(H_EMT_to_EMCS_sOne_interp(1:3,4,:),[1 3 2]), 'weights', weightsVector);
end
disp('Y_fake: ')
disp(T.M);
Y_fake = T.M;

for i = 1:size(H_OT_to_OCS_interp, 3)
    H_OT_to_OCS_interp(:,:,i) =  Y_fake * H_OT_to_OCS_interp(:,:,i);
end


H_OT_to_EMCS_cell_fake{1} = H_OT_to_OCS_interp;
H_EMT_to_EMCS_sOne_interp_cell{1} = H_EMT_to_EMCS_sOne_interp;

Plot_points(H_OT_to_EMCS_cell_fake,Plot_points(H_EMT_to_EMCS_sOne_interp_cell,[],1,'x'),3,'o');

% ICP
[H_EMT_to_EMCS_sOne_cell] = trackingdata_to_matrices(dataEM(:,1), 'ndi');
[H_OT_to_OCS] = trackingdata_to_matrices(dataOT(:,1), 'cpp');

H_OT_to_EMCS_fake_cell = cell(1);
m = size(H_OT_to_OCS{1}, 3);
for i = 1:m
    H_OT_to_EMCS_fake_cell{1}(:,:,i) =  Y_fake * H_OT_to_OCS{1}(:,:,i);
end

[R, T, Err] = icp(permute(H_EMT_to_EMCS_sOne_cell{1}(1:3,4,:),[1 3 2]), permute(H_OT_to_EMCS_fake_cell{1}(1:3,4,:),[1 3 2]), 'Weight', @(x)weightFcn(x, H_OT_to_EMCS_fake_cell{1}))
H = [R, T; 0 0 0 1];

for i = 1:m
    H_OT_to_EMCS_fake_cell{1}(:,:,i) =  H * H_OT_to_EMCS_fake_cell{1}(:,:,i);
end

Y_fake_corr = H*Y_fake;
disp('Y_fake_corr = ')
disp(Y_fake_corr)

Plot_points(H_OT_to_EMCS_fake_cell,Plot_points(H_EMT_to_EMCS_sOne_cell,[],1,'x'),3,'o');

% calculate all distances and find closest neighbor
tmpEM = permute(H_EMT_to_EMCS_sOne_cell{1}(1:3,4,:),[1 3 2])'; % nx3 matrix
many_EMT_positions = repmat(tmpEM,m,1); % m*nx3 matrix

n = size(H_EMT_to_EMCS_sOne_cell{1}, 3);
tmpOTx = permute(H_OT_to_EMCS_fake_cell{1}(1,4,:),[1 3 2]); % 1xm matrix
tmpOTy = permute(H_OT_to_EMCS_fake_cell{1}(2,4,:),[1 3 2]); % 1xm matrix
tmpOTz = permute(H_OT_to_EMCS_fake_cell{1}(3,4,:),[1 3 2]); % 1xm matrix
tmpOTx = repmat(tmpOTx,n,1); % n*m matrix
tmpOTy = repmat(tmpOTy,n,1); % n*m matrix
tmpOTz = repmat(tmpOTz,n,1); % n*m matrix

tmpDist = sqrt((many_EMT_positions(:,1)-tmpOTx(:)).^2 + (many_EMT_positions(:,2)-tmpOTy(:)).^2 + (many_EMT_positions(:,3)-tmpOTz(:)).^2);
clear many_EMT_positions tmpOTx tmpOTy tmpOTz
tmpDist = reshape(tmpDist,n,m);
[~,EM_ind_min] = min(tmpDist,[],1);
clear tmpDist

ClockOffsets = zeros(m,1);
for i = 1:m
    ClockOffsets(i) = dataEM{EM_ind_min(i),1}.DeviceTimeStamp - dataOT{i}.DeviceTimeStamp;
end
movementIndices = weightFcn(1:m, H_OT_to_EMCS_fake_cell{1}) > 0;
filteredClockOffsets = ClockOffsets(movementIndices);
% figure; hist(filteredClockOffsets, 100)
% figure; plot(filteredClockOffsets)
disp('Calculated offset. Add this to the NDI OT timestamps to be in Ascension time or subtract it from Ascension Device Timestamps to be in NDI time.')
offset = median(filteredClockOffsets);
disp(offset)
figure; hist(filteredClockOffsets-offset, 100)
title('Filtered Offset between the two device Clocks, median was removed.')



% Y_fake = T.M;
% 
% H_OT_to_EMCS_cell_fake{1} = H_OT_to_OCS_interp;
% H_EMT_to_EMCS_sOne_interp_cell{1} = H_EMT_to_EMCS_sOne_interp;
% 
% AscensionEMCS_fig = Plot_points(H_EMT_to_EMCS_sOne_interp_cell,[],1,'x');
% Plot_points(H_OT_to_EMCS_cell_fake,AscensionEMCS_fig,3,'o');

