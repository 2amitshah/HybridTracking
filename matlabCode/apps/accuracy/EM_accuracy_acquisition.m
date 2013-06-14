%This function computes the accuracy of EM using the EM and OT data
%computed at synthetic timestamps by
%OT_common_EMT_at_synthetic_timestamps.m
% 
% INPUT:
% path: where the EM and OT recording files are placed (with a common
% header)
% 
% H_OT_to_EMT: computed matrix which transforms the probe from OT to EM
% coordinate system. 4x4 containing rotation and translation
% 
% OUTPUT:
% 
% accuracy_cell: cell with two structs (taking into account simulated
% "blind" values, and only the real gathered data) with statistical
% parameters of the results, such as mean, maxima and minima, and standard
% deviation
% 
% Authors: Nicola Leucht, Santiago P�rez, June 2013
function accuracy_cell = EM_accuracy_acquisition(path, H_OT_to_EMT)
% variables definition
close all;
accuracy_cell = cell(1,2); %valid and not valid, mean, max, standard dev, RMS 

 if ~exist('path','var')
     pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
     path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
 end
 if ~exist('H_OT_to_EMT','var')
     load(which('H_OT_to_EMT.mat'));
 end
 
% get Y, equal to EMCS_to_OCS
Y = polaris_to_aurora(path, H_OT_to_EMT,'cpp');
 
%% get the improved position of EM 1 and EM 2 (and EM 3, if available) at the position of EM 1
% (data_EM_common) and the data of OT (data_OT_common) at the same synthetic timestamps
testrow_name_EMT = 'EMTrackingcont_2';
testrow_name_OT = 'OpticalTrackingcont_2';

[~, ~, data_EM_common, data_OT_common] =  OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT,testrow_name_OT);

%prepare data
numPts = size(data_EM_common,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

% Relevant matrix for computing transformations
[H_EMT_to_EMCS H_EMCS_to_EMT] = trackingdata_to_matrices(data_EM_common, 'CppCodeQuat');
[H_OT_to_OCS H_OCS_to_OT] = trackingdata_to_matrices(data_OT_common, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_OCS_to_OT = H_OCS_to_OT{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};
H_EMCS_to_EMT = H_EMCS_to_EMT{1,1};


%% calculate where EM tracker should be
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT; %data_EM_common_by_OT
end
data_EM_common_by_OT = cell(size(data_EM_common,1),1);
for i = 1:size(data_EM_common,1)
   data_EM_common_by_OT{i}.TimeStamp = data_OT_common{i}.TimeStamp;      
   data_EM_common_by_OT{i}.position = transpose(H_EMT_to_EMCS_by_OT(1:3,4,i) / H_EMT_to_EMCS_by_OT(4,4,i)) ;
   data_EM_common_by_OT{i}.orientation = transpose(rot2quat_q41(H_EMT_to_EMCS_by_OT(1:3, 1:3, i);));   
   data_EM_common_by_OT{i}.valid = data_OT_common{i}.valid;
end

%% compare data_EM_common_by_OT and data_EM_common
comparison_EM = cell(size(data_EM_common,1),1);
for i = 1:size(data_EM_common,1)
   comparison_EM{i}.position = norm(data_EM_common_by_OT{i}.position - data_EM_common{i}.position);
   comparison_EM{i}.orientation = norm(data_EM_common_by_OT{i}.orientation - data_EM_common{i}.orientation);
   comparison_EM{i}.valid = 0;
   if(data_EM_common_by_OT{i}.valid == 1 && data_EM_common{i}.valid == 1) % Only valid if both sensors have received proper data
       comparison_EM{i}.valid = 1;
   end
end

%% Obtaining statistical values
normOrientationOnlyValids = [];
normOrientation = zeros(1,size(comparison_EM,1));
normPositionOnlyValids = [];
normPosition = zeros(1,size(comparison_EM,1));
for i = 1:size(comparison_EM,1)
    normOrientation(i) = comparison_EM{i}.orientation;
    normPosition(i) = comparison_EM{i}.position;
    if (comparison_EM{i}.valid == 1)
        normOrientationOnlyValids(end+1) = comparison_EM{i}.orientation;
        normPositionOnlyValids(end+1) = comparison_EM{i}.position;
    end
end

plotEnvironment(3, H_OT_to_EMT, Y)

accuracy_cell{1}.orientation.mean = mean(normOrientation);
accuracy_cell{2}.orientation.mean = mean(normOrientationOnlyValids);
accuracy_cell{1}.orientation.max = max(normOrientation);
accuracy_cell{2}.orientation.max = max(normOrientationOnlyValids);
accuracy_cell{1}.orientation.min = min(normOrientation);
accuracy_cell{2}.orientation.min = min(normOrientationOnlyValids);
accuracy_cell{1}.orientation.std = std(normOrientation);
accuracy_cell{2}.orientation.std = std(normOrientationOnlyValids);
accuracy_cell{1}.position.mean = mean(normPosition);
accuracy_cell{2}.position.mean = mean(normPositionOnlyValids);
accuracy_cell{1}.position.max = max(normPosition);
accuracy_cell{2}.position.max = max(normPositionOnlyValids);
accuracy_cell{1}.position.min = min(normPosition);
accuracy_cell{2}.position.min = min(normPositionOnlyValids);
accuracy_cell{1}.position.std = std(normPosition);
accuracy_cell{2}.position.std = std(normPositionOnlyValids);

end