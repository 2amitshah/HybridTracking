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
% testrow_name_EMT: header of EMT continuous measurement file.
% 
% testrow_name_OT: header of OT continuous measurement file.
% 
% OUTPUT:
% 
% accuracy_cell: cell with two structs (taking into account simulated
% "blind" values, and only the real gathered data) with statistical
% parameters of the results, such as mean, maxima and minima, and standard
% deviation
%
% H_diff_EMT_to_EMCS: matrices which show the difference between the computed
% and the real EMT value
% 
% Authors: Nicola Leucht, Santiago Pérez, June 2013
function [accuracy_cell_corrected, accuracy_cell, translation_EMTcell, H_diff_EMT_to_EMCS] = EM_accuracy_acquisition_withwithout_correction(path, testrow_name_EMT, testrow_name_OT, H_OT_to_EMT)
% variables definition
close all;
accuracy_cell = cell(1,2); %valid and not valid, mean, max, standard dev, RMS 
accuracy_cell_corrected = cell(1,2); %valid and not valid, mean, max, standard dev, RMS 

 if ~exist('path','var')
     pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
     %path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
     path = [pathGeneral filesep 'measurements' filesep '07.16_Measurements'];
     %pathStatic = [pathGeneral filesep 'measurements' filesep '07.11_Measurements' filesep 'static positions'];

 end
 if ~exist('H_OT_to_EMT','var')
     load(which('H_OT_to_EMT.mat'));
 end
 
 if ~exist('testrow_name_EMT','var')
    testrow_name_EMT = 'EMTracking_newsd1';
    %testrow_name_EMT = 'EMTracking_distortionmap';
    
 end
 
 if ~exist('testrow_name_OT','var')
    testrow_name_OT = 'OpticalTracking_newsd1';
    %testrow_name_OT = 'OpticalTracking_distortionmap';
    
 end
 
 
% get Y, equal to EMCS_to_OCS
%[Y,H_OT_to_EMT] = polaris_to_aurora_absor(path, H_OT_to_EMT,'cpp','static','vRelease');
%Y = polaris_to_aurora_absor(pathStatic, H_OT_to_EMT,'cpp','static','vRelease');
Y = polaris_to_aurora_absor(path, H_OT_to_EMT,'cpp','dynamic','vRelease');
 
%% get the improved position of EM 1 and EM 2 (and EM 3, if available) at the position of EM 1
% (data_EM_common) and the data of OT (data_OT_common) at the same synthetic timestamps

[~, ~, data_EM_common, data_OT_common] =  OT_common_EMT_at_synthetic_timestamps_distortion_correction(path, testrow_name_EMT,testrow_name_OT, 20, 'vRelease');

%prepare data
numPts = size(data_EM_common,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

% Relevant matrix for computing transformations
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM_common, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT_common, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};


% calculate where EM tracker should be
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
H_diff_EMT_to_EMCS = zeros(4,4,numPts);
translation_EMTcell = cell(1,2);

data_EM_common_by_OT = cell(numPts,1);

for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT; %data_EM_common_by_OT
    data_EM_common_by_OT{i}.valid = 0;
    if(data_OT_common{i}.valid == 1 && data_EM_common{i}.valid == 1)
        H_diff_EMT_to_EMCS(:,:,i) = inv(H_EMT_to_EMCS(:,:,i))*H_EMT_to_EMCS_by_OT(:,:,i);
        translation_EMTcell{1}.vector(:,i) = H_EMT_to_EMCS_by_OT(1:3,4,i);
        translation_EMTcell{2}.vector(:,i) = H_EMT_to_EMCS(1:3,4,i);
    
        data_EM_common_by_OT{i}.TimeStamp = data_OT_common{i}.TimeStamp;      
        data_EM_common_by_OT{i}.position = transpose(H_EMT_to_EMCS_by_OT(1:3,4,i)) ;
        data_EM_common_by_OT{i}.orientation = transpose(rot2quat_q41(H_EMT_to_EMCS_by_OT(1:3, 1:3, i)));   
        data_EM_common_by_OT{i}.valid = data_OT_common{i}.valid;
    end
end

% compare data_EM_common_by_OT and data_EM_common
comparison_EM = cell(size(data_EM_common,1),1);
for i = 1:size(data_EM_common,1)
   comparison_EM{i}.valid = 0;
   if(data_EM_common_by_OT{i}.valid == 1 && data_EM_common{i}.valid == 1) % Only valid if both sensors have received proper data
    comparison_EM{i}.position = norm(data_EM_common_by_OT{i}.position - data_EM_common{i}.position);
    comparison_EM{i}.orientation = norm(data_EM_common_by_OT{i}.orientation - data_EM_common{i}.orientation);
    comparison_EM{i}.valid = 1;
   end
end

% Obtaining statistical values
normOrientationOnlyValids = [];
normOrientation = zeros(1,size(comparison_EM,1));
normPositionOnlyValids = [];
normPosition = zeros(1,size(comparison_EM,1));
for i = 1:size(comparison_EM,1)
    if (comparison_EM{i}.valid == 1)
        normOrientation(i) = comparison_EM{i}.orientation;
        normPosition(i) = comparison_EM{i}.position;
        normOrientationOnlyValids(end+1) = comparison_EM{i}.orientation;
        normPositionOnlyValids(end+1) = comparison_EM{i}.position;
    end
end


% plotEnvironment(3, H_OT_to_EMT, Y)
% title('What does this show?')

% plotEnvironment(4, H_OT_to_EMT, Y);
% Hmatrix_EM = trackingdata_to_matrices(data_EM_common,'CppCodeQuat');
Hmatrix_EM_by_OT = trackingdata_to_matrices(data_EM_common_by_OT, 'CppCodeQuat');
H_EMT_to_EMCS_cell{1} = H_EMT_to_EMCS;

EMTByOTfigure = figure;
Plot_points(H_EMT_to_EMCS_cell,EMTByOTfigure,1);
Plot_points(Hmatrix_EM_by_OT,EMTByOTfigure,2);
title('EMT_common (blue) and EMT_by_OT (green), WITH distortion correction')


accuracy_cell_corrected{1}.orientation.mean = mean(normOrientation);
accuracy_cell_corrected{2}.orientation.mean = mean(normOrientationOnlyValids);
accuracy_cell_corrected{1}.orientation.max = max(normOrientation);
accuracy_cell_corrected{2}.orientation.max = max(normOrientationOnlyValids);
accuracy_cell_corrected{1}.orientation.min = min(normOrientation);
accuracy_cell_corrected{2}.orientation.min = min(normOrientationOnlyValids);
accuracy_cell_corrected{1}.orientation.std = std(normOrientation);
accuracy_cell_corrected{2}.orientation.std = std(normOrientationOnlyValids);
accuracy_cell_corrected{1}.position.mean = mean(normPosition);
accuracy_cell_corrected{2}.position.mean = mean(normPositionOnlyValids);
accuracy_cell_corrected{1}.position.max = max(normPosition);
accuracy_cell_corrected{2}.position.max = max(normPositionOnlyValids);
accuracy_cell_corrected{1}.position.min = min(normPosition);
accuracy_cell_corrected{2}.position.min = min(normPositionOnlyValids);
accuracy_cell_corrected{1}.position.std = std(normPosition);
accuracy_cell_corrected{2}.position.std = std(normPositionOnlyValids);
accuracy_cell_corrected{1}.position.rms = sqrt(sum(normPosition.^2) / size(comparison_EM,1));
accuracy_cell_corrected{1}.position.rms = sqrt(sum(normPositionOnlyValids.^2) / size(comparison_EM,1));


disp(['Mean position: ' num2str(accuracy_cell_corrected{1}.position.mean)]);
disp(['Mean position only valids: ' num2str(accuracy_cell_corrected{2}.position.mean)]);

% distance output
deviation_length_collector=zeros(1,numPts);
for i = 1:numPts
    deviation_length_collector(i)=norm(H_diff_EMT_to_EMCS(1:3,4,i));
end
disp 'deviation of EMT due to field errors'
disp(mean(deviation_length_collector))

normPositionCorrected = normPosition;


%% without distortion correction
[~, ~, data_EM_common, data_OT_common] =  OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT,testrow_name_OT, 20, 'vRelease');

%prepare data
numPts = size(data_EM_common,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

% Relevant matrix for computing transformations
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM_common, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT_common, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};



% calculate where EM tracker should be
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
H_diff_EMT_to_EMCS = zeros(4,4,numPts);
translation_EMTcell = cell(1,2);

data_EM_common_by_OT = cell(numPts,1);

for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT; %data_EM_common_by_OT
    data_EM_common_by_OT{i}.valid = 0;
    if(data_OT_common{i}.valid == 1 && data_EM_common{i}.valid == 1)
        H_diff_EMT_to_EMCS(:,:,i) = inv(H_EMT_to_EMCS(:,:,i))*H_EMT_to_EMCS_by_OT(:,:,i);
        translation_EMTcell{1}.vector(:,i) = H_EMT_to_EMCS_by_OT(1:3,4,i);
        translation_EMTcell{2}.vector(:,i) = H_EMT_to_EMCS(1:3,4,i);
    
        data_EM_common_by_OT{i}.TimeStamp = data_OT_common{i}.TimeStamp;      
        data_EM_common_by_OT{i}.position = transpose(H_EMT_to_EMCS_by_OT(1:3,4,i)) ;
        data_EM_common_by_OT{i}.orientation = transpose(rot2quat_q41(H_EMT_to_EMCS_by_OT(1:3, 1:3, i)));   
        data_EM_common_by_OT{i}.valid = data_OT_common{i}.valid;
    end
end

% compare data_EM_common_by_OT and data_EM_common
comparison_EM = cell(size(data_EM_common,1),1);
for i = 1:size(data_EM_common,1)
   comparison_EM{i}.valid = 0;
   if(data_EM_common_by_OT{i}.valid == 1 && data_EM_common{i}.valid == 1) % Only valid if both sensors have received proper data
    comparison_EM{i}.position = norm(data_EM_common_by_OT{i}.position - data_EM_common{i}.position);
    comparison_EM{i}.orientation = norm(data_EM_common_by_OT{i}.orientation - data_EM_common{i}.orientation);
    comparison_EM{i}.valid = 1;
   end
end

% Obtaining statistical values
normOrientationOnlyValids = [];
normOrientation = zeros(1,size(comparison_EM,1));
normPositionOnlyValids = [];
normPosition = zeros(1,size(comparison_EM,1));
for i = 1:size(comparison_EM,1)
    if (comparison_EM{i}.valid == 1)
        normOrientation(i) = comparison_EM{i}.orientation;
        normPosition(i) = comparison_EM{i}.position;
        normOrientationOnlyValids(end+1) = comparison_EM{i}.orientation;
        normPositionOnlyValids(end+1) = comparison_EM{i}.position;
    end
end


% plotEnvironment(3, H_OT_to_EMT, Y)
% title('What does this show?')

% plotEnvironment(4, H_OT_to_EMT, Y);
% Hmatrix_EM = trackingdata_to_matrices(data_EM_common,'CppCodeQuat');
Hmatrix_EM_by_OT = trackingdata_to_matrices(data_EM_common_by_OT, 'CppCodeQuat');
H_EMT_to_EMCS_cell{1} = H_EMT_to_EMCS;

EMTByOTfigure_nocorr = figure;
Plot_points(H_EMT_to_EMCS_cell,EMTByOTfigure_nocorr,1);
Plot_points(Hmatrix_EM_by_OT,EMTByOTfigure_nocorr,2);
title('EMT_commom (blue) and EMT_by_OT (green), without distortion correction')


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
accuracy_cell{1}.position.rms = sqrt(sum(normPosition.^2) / size(comparison_EM,1));
accuracy_cell{1}.position.rms = sqrt(sum(normPositionOnlyValids.^2) / size(comparison_EM,1));


disp(['Mean position: ' num2str(accuracy_cell{1}.position.mean)]);
disp(['Mean position only valids: ' num2str(accuracy_cell{2}.position.mean)]);

% distance output
deviation_length_collector=zeros(1,numPts);
for i = 1:numPts
    deviation_length_collector(i)=norm(H_diff_EMT_to_EMCS(1:3,4,i));
end
disp 'deviation of EMT due to field errors'
disp(mean(deviation_length_collector))

boxplotfigure = figure;
boxplot([normPosition', normPositionCorrected'],'label',{'original positions','corrected positions'});
end