function [Fu, Fv, Fw] = distortion_nicola(path, H_OT_to_EMT, testrow_name_EMT, testrow_name_OT)
close all
 if ~exist('path','var')
     pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
     %path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
     path = [pathGeneral filesep 'measurements' filesep '07.11_Measurements'];
 end
 if ~exist('H_OT_to_EMT','var')
     load(which('H_OT_to_EMT.mat'));
 end
  if ~exist('testrow_name_EMT','var')
    %testrow_name_EMT = 'EMTrackingcont_1';
    %testrow_name_EMT = 'EMTracking_distortionmap';
    testrow_name_EMT = 'EMTracking_firstVolume';
 end
 
 if ~exist('testrow_name_OT','var')
    %testrow_name_OT = 'OpticalTrackingcont_1';
    %testrow_name_OT = 'OpticalTracking_distortionmap';
    testrow_name_OT = 'OpticalTracking_firstVolume';
 end
%% get Y
pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
pathStatic = [pathGeneral filesep 'measurements' filesep '07.11_Measurements' filesep 'static positions'];
[Y,H_OT_to_EMT] = polaris_to_aurora_absor(pathStatic, H_OT_to_EMT,'cpp','static','vRelease');
%Y = polaris_to_aurora_absor(path, H_OT_to_EMT,'cpp','static','vRelease');
 
%% get positions
frequency = 10;
[~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT,testrow_name_OT, frequency, 'vRelease');

for i = 1:size(data_EMT,1)
    if ~data_EMT{i}.valid
        data_EMT{i} = data_EMT{i-1};
    end
end
for i = 1:size(data_OT,1)
    if ~data_OT{i}.valid
        data_OT{i} = data_OT{i-1};
    end
end

%prepare data
numPts = size(data_EMT,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

for i = 1:numPts

    %insert rotation into homogeneous matrix
    mat{1}(:,:,i) = quat2rot((data_EMT{i,1}.orientation(1:3))');

    mat{2}(:,:,i) = quat2rot((data_OT{i}.orientation(1:3))');

    %add translation
    mat{1}(:,:,i) = transl(data_EMT{i,1}.position') * mat{1}(:,:,i);

    mat{2}(:,:,i) = transl(data_OT{i}.position') * mat{2}(:,:,i);

end

%gHc = tracking_handEyeCalib(bHg, wHc)

% We want to have EMCS as world coordinates. World in Tsai's setup always
% means the grid. I will filp this to match EMT~Cam and OT~Marker.
for i = 1:numPts
    H_EMCS_to_EMT(:,:,i)    = inv(mat{1}(:,:,i)); %(=Hgrid2cam)
    H_EMT_to_EMCS(:,:,i)    = mat{1}(:,:,i);
    H_OT_to_OCS(:,:,i)      = mat{2}(:,:,i);      %(=Hmarker2world)
    H_OCS_to_OT(:,:,i)      = inv(mat{2}(:,:,i));
end

%% calculate OT in EMCS coordiantes using Y
for i = 1:numPts
    %optical
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    %rotation
    xaxes_optical_in_EMCS(i,:) = H_OT_to_EMCS(1:3,1)';
    yaxes_optical_in_EMCS(i,:) = H_OT_to_EMCS(1:3,2)';
    zaxes_optical_in_EMCS(i,:) = H_OT_to_EMCS(1:3,3)';
    %translation
    opticalPoints_in_EMCS(:,i) = H_OT_to_EMCS(1:3,4);
    
    %em tracker
    %rotation
    xaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,1,i)';
    yaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,2,i)';
    zaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,3,i)';
    %translation
    emPointsFirstSensor(:,i) = H_EMT_to_EMCS(1:3,4,i);
end

%% plot position data
% plot to see how positions were recorded
c = colormap('lines');
close(gcf);
% plot all OT_in_EMCS positions

r_sphere = 4;
[Xsp, Ysp, Zsp] = sphere;
Xsp = Xsp * r_sphere;
Ysp = Ysp * r_sphere;
Zsp = Zsp * r_sphere;

pathfig = figure;
hold on
plot3(opticalPoints_in_EMCS(1,:), opticalPoints_in_EMCS(2,:), opticalPoints_in_EMCS(3,:), 'x', 'Color', c(1,:) );
hold off
% for i = 1:numPts
%     % plot little sphere in grey to look like the OT
%     hold on
%     surf(Xsp+opticalPoints_in_EMCS(1,i), Ysp+opticalPoints_in_EMCS(2,i), Zsp+opticalPoints_in_EMCS(3,i), 'EdgeColor', 'none', 'FaceColor', [.9 .9 .9], 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
%     hold off
% end
title({'Optical Center position in optical coordinate system OCS',...
    'orientation is shown in XYZ = RGB'})
xlabel('x')
ylabel('y')
zlabel('z')

% plot all EMT positions
figure(pathfig)
hold on
plot3(emPointsFirstSensor(1,:), emPointsFirstSensor(2,:),emPointsFirstSensor(3,:), 'x', 'Color', c(1,:) );
hold off
title({'EM Tracker position in electromagnetical coordinate system (EMCS)',...
    'orientation is shown in XYZ = RGB',...
    'transformed OT position in EMCS is shown as red x, corresponding positions are connected'})
xlabel('x')
ylabel('y')
zlabel('z')


set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')

axis image vis3d
camlight('headlight');

%% calculate where EM tracker should be
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT;
end
%% plot where EM tracker should be
for i = 1:numPts
    %em tracker
    %rotation
    xaxes_emFirst_by_OT(i,:) = H_EMT_to_EMCS_by_OT(1:3,1,i)';
    yaxes_emFirst_by_OT(i,:) = H_EMT_to_EMCS_by_OT(1:3,2,i)';
    zaxes_emFirst_by_OT(i,:) = H_EMT_to_EMCS_by_OT(1:3,3,i)';
    %translation
    emPointsFirstSensor_by_OT(:,i) = H_EMT_to_EMCS_by_OT(1:3,4,i);
end
% plot all EMT positions by OT
figure(pathfig)
hold on
plot3(emPointsFirstSensor_by_OT(1,:), emPointsFirstSensor_by_OT(2,:),emPointsFirstSensor_by_OT(3,:), 'x', 'Color', c(1,:), 'MarkerSize', 10 );
hold off



%% distance output
H_diff_EMT = zeros(4,4,numPts);
for i = 1:numPts
    H_diff_EMT(:,:,i) = inv(H_EMT_to_EMCS(:,:,i))*H_EMT_to_EMCS_by_OT(:,:,i);
    disp 'deviation of EMT due to field errors'
    norm(H_diff_EMT(1:3,4,i))
end

% plot the aurora table
plotAuroraTable(pathfig);
axis image vis3d

%% check the correct direction
hold on
arrow3(emPointsFirstSensor_by_OT',emPointsFirstSensor', 'k', 0.1, 0.1, [], .6)
hold off

view(3)
%% interpolate vector field

xdiff = zeros(1,numPts);
ydiff = zeros(1,numPts);
zdiff = zeros(1,numPts);
for i = 1:numPts
    % difference vectors are computed pointing from measured EMT position to
    % where EMT sould have been due to OT
    xdiff(i) = emPointsFirstSensor(1,i) - emPointsFirstSensor_by_OT(1,i);
    ydiff(i) = emPointsFirstSensor(2,i) - emPointsFirstSensor_by_OT(2,i);
    zdiff(i) = emPointsFirstSensor(3,i) - emPointsFirstSensor_by_OT(3,i);
end
Fu = scatteredInterpolant(emPointsFirstSensor', xdiff', 'natural', 'nearest'); %was 'none' instead of 'linear' %difference in x-direction at position xyz, xyz defined by empointsfirst...
Fv = scatteredInterpolant(emPointsFirstSensor', ydiff', 'natural', 'nearest'); %difference in y-direction at position xyz
Fw = scatteredInterpolant(emPointsFirstSensor', zdiff', 'natural', 'nearest'); %difference in z-direction at position xyz


%positions at which i want to know the vector values
minx = min(emPointsFirstSensor(1,:));
maxx = max(emPointsFirstSensor(1,:));
miny = min(emPointsFirstSensor(2,:));
maxy = max(emPointsFirstSensor(2,:));
minz = min(emPointsFirstSensor(3,:));
maxz = max(emPointsFirstSensor(3,:));
%[Xi, Yi, Zi] = meshgrid(-250:50:250,-300:50:300,-500:50:-100);
% [Xi, Yi, Zi] = meshgrid(-100:10:100,-100:10:150,-250:5:-150);

[Xi, Yi, Zi] = meshgrid(minx:20:maxx,miny:20:maxy,minz:10:maxz);


Ui = Fu(Xi, Yi, Zi); %Ui is difference in x-direction at the point xi, yi, zi
Vi = Fv(Xi, Yi, Zi);
Wi = Fw(Xi, Yi, Zi);

% plot Distortion vector

vectorfig = figure;

distortionNorm = sqrt(Ui.^2+Vi.^2+Wi.^2);
indicesHighDistortion = find(distortionNorm > median(median(median(distortionNorm))));
quiver3(Xi(indicesHighDistortion),Yi(indicesHighDistortion),Zi(indicesHighDistortion),...
		Ui(indicesHighDistortion),Vi(indicesHighDistortion),Wi(indicesHighDistortion))

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')
title({'Distortion vector field'})%,...
%     '2nd line',...
%     '3rd line'})
xlabel('x')
ylabel('y')
zlabel('z')
plotAuroraTable(vectorfig);
axis image vis3d

slicefig = figure;
% figure(pathfig);
hold on
h = slice(Xi, Yi, Zi, distortionNorm,[-200:200],[-150:150],[-300:10:-50]);
%h = slice(Xi, Yi, Zi, distortionNorm,[],[],[minz:10:maxz]);
%h = slice(Xi, Yi, Zi, distortionNorm,[-400:400],[-400:400],[-250:20:50]

set(h,'FaceColor','interp',...
	'EdgeColor','none',...
	'DiffuseStrength',.8,'FaceAlpha', 0.2);
hold off
set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')
title({'Distortion vectorlength field'})%,...
%     '2nd line',...
%     '3rd line'})
xlabel('x')
ylabel('y')
zlabel('z')

plotAuroraTable(slicefig);
plotAuroraVolume(slicefig);
axis image vis3d

disp 'durchschnittlicher interpolierter fehler'
UVE_Len_notNan = distortionNorm(isfinite(distortionNorm));
UVE_Len_notNan = UVE_Len_notNan(:);
mean(UVE_Len_notNan)

end