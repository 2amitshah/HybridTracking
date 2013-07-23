%% USE CASE
% Distortion correction. Tries to correct the distorted measurements of one
% experiment with the gathered values in a previous analysis.
% Author: Felix Achilles, July 2013

close all; clear all; clc

y_computation;

filenames_struct.EMfiles = 'EMTracking_newsd1';
filenames_struct.OTfiles = 'OpticalTracking_newsd1';
[Fu, Fv, Fw] = distortion_new(filenames_struct,'vBlack', Y);
clc;

testrow_name_EMT = 'EMTracking_newsd2';
testrow_name_OT = 'OpticalTracking_newsd2';

frequency = 20;
[~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT,testrow_name_OT, frequency, 'vRelease');
data_EMT_corrected = distortion_correction(data_EMT, Fu, Fv, Fw);


% Relevant matrix for computing transformations
[H_EMT_to_EMCS] =           trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
[H_EMT_to_EMCS_corrected] = trackingdata_to_matrices(data_EMT_corrected, 'CppCodeQuat');
[H_OT_to_OCS] =             trackingdata_to_matrices(data_OT, 'CppCodeQuat');

H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};
H_EMT_to_EMCS_corrected = H_EMT_to_EMCS_corrected{1};
H_OT_to_OCS = H_OT_to_OCS{1,1};

% calculate where EM tracker should be
load('H_OT_to_EMT')
numPts = size(H_EMT_to_EMCS, 3);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
normPosition = [];
normPositionCorrected = [];

for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS / H_OT_to_EMT; %data_EM_common_by_OT
    
    if(data_OT{i}.valid == 1 && data_EMT{i}.valid == 1)
        H_diff_EMT_to_EMCS(:,:) = (H_EMT_to_EMCS(:,:,i))\H_EMT_to_EMCS_by_OT(:,:,i);
        normPosition = [normPosition norm(H_diff_EMT_to_EMCS(1:3,4))];
        H_diff_EMT_to_EMCS_corrected(:,:) = (H_EMT_to_EMCS_corrected(:,:,i))\H_EMT_to_EMCS_by_OT(:,:,i);
        normPositionCorrected = [normPositionCorrected norm(H_diff_EMT_to_EMCS_corrected(1:3,4))];
    end
end
figure;
boxplot([normPosition', normPositionCorrected'],'label',{'original positions','corrected positions'});
title('whassuuuuup')