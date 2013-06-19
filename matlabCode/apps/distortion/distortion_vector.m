%This function computes the plots the distortion field vector
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
% Authors: Felix Achilles, Santiago Pérez, June 2013

function distortion_vector(path, testrow_name_EMT, testrow_name_OT, H_OT_to_EMT)

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
 
 if ~exist('testrow_name_EMT','var')
    testrow_name_EMT = 'EMTrackingcont_screwdriver_2';
 end
 
 if ~exist('testrow_name_OT','var')
    testrow_name_OT = 'OpticalTrackingcont_screwdriver_2';
 end
 
 [~, EMTvectorcell, distortion_matrix] = EM_accuracy_acquisition(path, testrow_name_EMT, testrow_name_OT, H_OT_to_EMT);
 
 
 %% check the correct direction
% try TriScatteredInterp
% use arrow3
% for i = 1:numPts
figure
    hold on
    arrow3(EMTvectorcell{1}.vector',EMTvectorcell{2}.vector', 'k', 1, 1, [], .6)
    hold off
% end
 
 
 
%% interpolate vector field
Fu = scatteredInterpolant(EMTvectorcell{1}.vector', permute(distortion_matrix(1,4,:),[3 2 1]), 'natural', 'none');
Fv = scatteredInterpolant(EMTvectorcell{1}.vector', permute(distortion_matrix(2,4,:),[3 2 1]), 'natural', 'none');
Fw = scatteredInterpolant(EMTvectorcell{1}.vector', permute(distortion_matrix(3,4,:),[3 2 1]), 'natural', 'none');

%positions at which i want to know the vector values
[Xi, Yi, Zi] = meshgrid(-250:50:250,-300:50:300,-500:50:-100);

% POS_i = [Xi(:),Yi(:),Zi(:)];
% Ui = Fu(POS_i);
% Vi = Fv(POS_i);
% Wi = Fw(POS_i);
    
% for i=1:size(POS_i,1)
    
% end

Ui = Fu(Xi, Yi, Zi);
Vi = Fv(Xi, Yi, Zi);
Wi = Fw(Xi, Yi, Zi);

% plot Distortion vector
figure;
quiver3(Xi, Yi, Zi, Ui, Vi, Wi)

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')
title({'Distortion vector field'})%,...
%     '2nd line',...
%     '3rd line'})
xlabel('x')
ylabel('y')
zlabel('z')
axis image vis3d

%vector length
UVW_Len = sqrt(Ui.^2 + Vi.^2 + Wi.^2);
figure;
slice(Xi, Yi, Zi, UVW_Len,[],[],[-500:50:-100])
set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')
title({'Distortion vectorlength field'})%,...
%     '2nd line',...
%     '3rd line'})
xlabel('x')
ylabel('y')
zlabel('z')
axis image vis3d

disp 'durchschnittlicher interpolierter fehler'
UVE_Len_notNan = UVW_Len(isfinite(UVW_Len));
UVE_Len_notNan = UVE_Len_notNan(:);
mean(UVE_Len_notNan);
 
 
 
 end