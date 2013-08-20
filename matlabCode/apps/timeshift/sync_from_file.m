function EM_minus_OT_offset = sync_from_file(filenames_struct, verbosity, TSOption)
% Tracking data read-in. Then a point-could-fit is used to guess the
% timestamp-offset.
% Author: Felix Achilles, July 2013

if ~exist('TSOption', 'var') || isempty(TSOption)
    TSOption = 'network';
end

% Polaris and Ascension
% [dataOT, dataEM] = read_Direct_OpticalAndAscension(filenames_struct, [], [], 'vRelease');

% Polaris and Aurora
[dataOT, dataEM] = read_Direct_NDI_PolarisAndAurora(filenames_struct, 'vRelease');

% Polaris and Ascension
% [H_EMT_to_EMCS_sOne_interp] = frame_interpolation(dataEM(:,1), [], 20, 'ndi', TSOption);

% Polaris and Aurora
[H_EMT_to_EMCS_sOne_interp] = frame_interpolation(dataEM(:,1), [], 20, 'cpp', TSOption);
[H_OT_to_OCS_interp] = frame_interpolation(dataOT(:,1), [], 20, 'cpp', TSOption);

numEMPts_interp = size(H_EMT_to_EMCS_sOne_interp, 3);
numOTPts_interp = size(H_OT_to_OCS_interp, 3);

% create a weights vector
% to not overweigh long start and end points, but regular movement
% positions inbetween
[minimal_num_pts, minIndx] = min([numEMPts_interp, numOTPts_interp]);
weightsVector = zeros(minimal_num_pts,1);
if minIndx == 1
    tmpFrames = H_EMT_to_EMCS_sOne_interp;
elseif minIndx == 2
    tmpFrames = H_OT_to_OCS_interp;
end
for j = 1:minimal_num_pts-1
    weightsVector(j) = norm(tmpFrames(1:3,4,j+1) - tmpFrames(1:3,4,j));
end
weightsVector(weightsVector > 10) = 0;
weightsVector = weightsVector.^2;

% feature: use generic weighting function that outputs the speed in mm/s
% and 0 if the movement was slower than 5 mm/s.

% fit the last points of the recordings to one another
if numEMPts_interp > numOTPts_interp
    T = absor(permute(H_OT_to_OCS_interp(1:3,4,:),[1 3 2]), permute(H_EMT_to_EMCS_sOne_interp(1:3,4,(end-size(H_OT_to_OCS_interp, 3)+1):end),[1 3 2]), 'weights', weightsVector);
else
    T = absor(permute(H_OT_to_OCS_interp(1:3,4,(end-size(H_EMT_to_EMCS_sOne_interp, 3)+1):end),[1 3 2]), permute(H_EMT_to_EMCS_sOne_interp(1:3,4,:),[1 3 2]), 'weights', weightsVector);
end
disp('Y_fake: ')
disp(T.M);
Y_fake = T.M;

H_OT_to_EMCS_fake = H_OT_to_OCS_interp;
for i = 1:numOTPts_interp
    H_OT_to_EMCS_fake(:,:,i) =  Y_fake * H_OT_to_OCS_interp(:,:,i);
end

if strcmp(verbosity, 'vDebug')
    H_OT_to_EMCS_cell_fake{1} = H_OT_to_EMCS_fake;
    H_EMT_to_EMCS_sOne_interp_cell{1} = H_EMT_to_EMCS_sOne_interp;

    Plot_points(H_OT_to_EMCS_cell_fake,Plot_points(H_EMT_to_EMCS_sOne_interp_cell,[],1,'x'),3,'o');
end

%Start with initial guess of interpolated data, then perform ICP on
%original data

% Polaris and Ascension
% [H_EMT_to_EMCS_sOne_cell] = trackingdata_to_matrices(dataEM(:,1), 'ndi');

% Polaris and Aurora
[H_EMT_to_EMCS_sOne_cell] = trackingdata_to_matrices(dataEM(:,1), 'cpp');
[H_OT_to_OCS] = trackingdata_to_matrices(dataOT(:,1), 'cpp');

H_OT_to_EMCS_fake_cell = cell(1);
m = size(H_OT_to_OCS{1}, 3);
for i = 1:m
    H_OT_to_EMCS_fake_cell{1}(:,:,i) =  Y_fake * H_OT_to_OCS{1}(:,:,i);
end

[R, T, Err] = icp(permute(H_EMT_to_EMCS_sOne_cell{1}(1:3,4,:),[1 3 2]), permute(H_OT_to_EMCS_fake_cell{1}(1:3,4,:),[1 3 2]), 'Weight', @(x)weightFcn(x, H_OT_to_EMCS_fake_cell{1}))
H_icp = [R, T; 0 0 0 1];

for i = 1:m
    H_OT_to_EMCS_fake_cell{1}(:,:,i) =  H_icp * H_OT_to_EMCS_fake_cell{1}(:,:,i);
end

Y_fake_corr = H_icp*Y_fake;
disp('Y_fake_corrected = ')
disp(Y_fake_corr)

if strcmp(verbosity, 'vDebug')
    Plot_points(H_OT_to_EMCS_fake_cell,Plot_points(H_EMT_to_EMCS_sOne_cell,[],1,'x'),3,'o');
end

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

% only use calculated clock offset of points taken during motion
movementIndices = weightFcn(1:m, H_OT_to_EMCS_fake_cell{1}) > 0;
filteredClockOffsets = ClockOffsets(movementIndices);
% Ascension Text
% disp('Calculated offset.\n Add this to the NDI OT timestamps to be in Ascension time\n or subtract it from Ascension Device Timestamps to be in NDI time.')
% Aurora Text
disp(['Calculated offset in seconds' char(10) ...
    'Add this to the NDI Polaris timestamps to be in NDI Aurora time' char(10) ...
    'or subtract it from NDI Aurora Device Timestamps to be in NDI Polaris time:'])
EM_minus_OT_offset = median(filteredClockOffsets);
disp([ num2str(EM_minus_OT_offset) ' seconds.'])
figure; hist(filteredClockOffsets-EM_minus_OT_offset, 100)
title('Filtered Offset between the two device Clocks, median was removed.')

end