%This function computes the accuracy of EM using the EM and OT data
%computed at synthetic timestamps by
%OT_common_EMT_at_synthetic_timestamps.m

function accuracy_cell = EM_accuracy_acquisition(path, H_OT_to_EMT)
close all;
accuracy_cell = cell(1,2); %valid and not valid, mean, max, standard dev, RMS 

 if ~exist('path','var')
     pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
     path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];
 end
 if ~exist('H_OT_to_EMT','var')
     load(which('H_OT_to_EMT.mat'));
 end
 
% get Y
Y = polaris_to_aurora(path, H_OT_to_EMT);
 
%% get the improved position of EM 1 and EM 2 (and EM 3, if available) at the position of EM 1
% (data_EM_common) and the data of OT (data_OT_common) at the same synthetic timestamps
testrow_name_EMT = 'EMTracking_cont';
testrow_name_OT = 'OpticalTracking_cont';

[~, ~, data_EM_common, data_OT_common] =  OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT);
%[frame, invframe, data_EM_common, data_OT_common] =  OT_common_EMT_at_synthetic_timestamps();

%prepare data
numPts = size(data_EM_common,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end


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
   R = H_EMT_to_EMCS_by_OT(1:3, 1:3, i);
   R = R ./ H_EMT_to_EMCS_by_OT(4,4,i);   
   data_EM_common_by_OT{i}.orientation = transpose(rot2quat_q41(R));   
   data_EM_common_by_OT{i}.valid = data_OT_common{i}.valid;
end

%% compare data_EM_common_by_OT and data_EM_common
comparison_EM = cell(size(data_EM_common,1),1);
for i = 1:size(data_EM_common,1)
   comparison_EM{i}.distance = norm(data_EM_common_by_OT{i}.position - data_EM_common{i}.position);
   comparison_EM{i}.orientation = norm(data_EM_common_by_OT{i}.orientation - data_EM_common{i}.orientation);
   comparison_EM{i}.valid = 0;
   if(data_EM_common_by_OT{i}.valid == 1 && data_EM_common{i}.valid == 1)
       comparison_EM{i}.valid = 1;
   end
end

% Obtaining statistical values



%
end