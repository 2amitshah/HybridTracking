% distortion_new
%
% Authors: Nicola Leucht, Santiago Perez, Felix Achilles
% July 2013

function [Fu, Fv, Fw, normcollector] = distortion_new(path, verbosity, Y)

load('H_OT_to_EMT');

if ~exist('verbosity','var')
    verbosity = 'vRelease';
end
if ~exist('path','var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    %path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
    path = [pathGeneral filesep 'measurements' filesep '07.11_Measurements'];
    % get default Y
    pathStatic = [path filesep 'static positions'];
    Y = polaris_to_aurora(pathStatic, H_OT_to_EMT,'cpp','static','vRelease');
end

filenames_struct = path;
if isstruct(filenames_struct)
    testrow_name_EMT = filenames_struct.EMfiles;
    testrow_name_OT = filenames_struct.OTfiles;
    path = filenames_struct.folder;
else
    warning('path is not a struct. default names for testrow_name_EMT and testrow_name_OT will be used. please update path file format to a struct.')
end

if ~exist('testrow_name_EMT','var')
    %testrow_name_EMT = 'EMTrackingcont_1';
    testrow_name_EMT = 'EMTracking_distortionmap';
end

if ~exist('testrow_name_OT','var')
    %testrow_name_OT = 'OpticalTrackingcont_1';
    testrow_name_OT = 'OpticalTracking_distortionmap';
end
%% get Y
if ~exist('Y','var')
    Y = polaris_to_aurora(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease');
end
 
%% get interpolated synchronous positions
frequency = 20;
[~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT,testrow_name_OT, frequency, 'vRelease');



% read out the .valid parameter and store in array
data_EMT_arraystruct = [data_EMT{:,1}];
data_EMT_valid_array = [data_EMT_arraystruct.valid];
data_OT_arraystruct = [data_OT{:}];
data_OT_valid_array = [data_OT_arraystruct.valid];

% create bool array that marks valid points
OT_and_EMT_valid = data_EMT_valid_array & data_OT_valid_array;

% update struct cells, only take .valid data that were taken at the same time
data_OT_arraystruct = [data_OT{OT_and_EMT_valid}]';
data_EMT_arraystruct = [data_EMT{OT_and_EMT_valid}]';
data_OT = num2cell(data_OT_arraystruct);
data_EMT = num2cell(data_EMT_arraystruct);

% clear memory
clear data_EMT_arraystruct data_OT_arraystruct

H_EMT_to_EMCS_cell = trackingdata_to_matrices(data_EMT,'cpp');
H_OT_to_OCS_cell = trackingdata_to_matrices(data_OT,'cpp');
H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
H_OT_to_OCS = H_OT_to_OCS_cell{1};
numPts = size(data_EMT,1);

%% calculate OT in EMCS coordiantes using Y
H_OT_to_EMCS_cell = cell(1,1);
for i = 1:numPts
    %optical
    H_OT_to_EMCS_cell{1}(:,:,i) = Y*H_OT_to_OCS(:,:,i);
end

%% plot position data

% TODO
% remove this, it is just for debugging
[data_OT_raw, data_EMT_raw] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);
H_EMT_to_EMCS_cell_raw = trackingdata_to_matrices(data_EMT_raw,'cpp');

% plot to see how positions were recorded
c = colormap('lines');
close(gcf);

% plot all OT_in_EMCS positions

r_sphere = 4;
[Xsp, Ysp, Zsp] = sphere;
Xsp = Xsp * r_sphere;
Ysp = Ysp * r_sphere;
Zsp = Zsp * r_sphere;



if (strcmp(verbosity, 'vDebug') || strcmp(verbosity, 'vEyecandy') || strcmp(verbosity, 'vRelease'))
    pathfig = figure;
    Plot_points(H_OT_to_EMCS_cell, pathfig, 3, 'x');
    Plot_points(H_EMT_to_EMCS_cell_raw, pathfig, 2, 'x');
    

% hold on
% plot3(opticalPoints_in_EMCS(1,:), opticalPoints_in_EMCS(2,:), opticalPoints_in_EMCS(3,:), 'rx')%, 'Color', c(1,:) );
% hold off

if strcmp(verbosity, 'vEyecandy')
for i = 1:numPts
    % plot little sphere in grey to look like the OT
    hold on
    surf(Xsp+opticalPoints_in_EMCS(1,i), Ysp+opticalPoints_in_EMCS(2,i), Zsp+opticalPoints_in_EMCS(3,i), 'EdgeColor', 'none', 'FaceColor', [.9 .9 .9], 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
    hold off
end
end
title({'Optical Center position in EMCS'})%,...
    %'orientation is shown in XYZ = RGB'})

% plot all EMT positions
Plot_points(H_EMT_to_EMCS_cell, pathfig, 1, 'x');

title({'EM Tracker position in electromagnetical coordinate system (EMCS)',...
    'blue x marks measured EM position, green x marks where it should have been',...
    'transformed OT position in EMCS is shown as red x'})%, corresponding positions are connected'})
end

%% calculate where EM tracker should be
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
for i = 1:numPts
    H_EMT_to_EMCS_by_OT(:,:,i) = Y*H_OT_to_OCS(:,:,i)/H_OT_to_EMT;
end

emPointsFirstSensor_by_OT = zeros(3,numPts);
emPointsFirstSensor = zeros(3,numPts);
for i = 1:numPts
    %em tracker
    %translation
    emPointsFirstSensor_by_OT(:,i) = H_EMT_to_EMCS_by_OT(1:3,4,i);
    emPointsFirstSensor(:,i) = H_EMT_to_EMCS(1:3,4,i);
end

%% plot all EMT positions by OT
if (strcmp(verbosity, 'vDebug') || strcmp(verbosity, 'vEyecandy') || strcmp(verbosity, 'vRelease'))
wrapper{1}=H_EMT_to_EMCS_by_OT;
Plot_points(wrapper, pathfig, 1, 'o');
end
%% distance output

H_diff_EMT = zeros(4,4,numPts);
normcollector = zeros(1,numPts);
for i = 1:numPts
    H_diff_EMT(:,:,i) = H_EMT_to_EMCS(:,:,i) \ H_EMT_to_EMCS_by_OT(:,:,i);
    normcollector(i) = norm(H_diff_EMT(1:3,4,i));
    if normcollector(i) > 40
        figure(pathfig)
        hold on
        line([emPointsFirstSensor_by_OT(1,i);emPointsFirstSensor(1,i)], [emPointsFirstSensor_by_OT(2,i);emPointsFirstSensor(2,i)], [emPointsFirstSensor_by_OT(3,i);emPointsFirstSensor(3,i)])
        hold off
    end
    if strcmp(verbosity, 'vDebug')
        disp 'deviation of EMT due to field errors:'
        norm(H_diff_EMT(1:3,4,i))
    end
end
if strcmp(verbosity, 'vRelease')
    disp 'durchschnittlicher Fehler an Pfad-Punkten:'
    disp(mean(normcollector));
    disp 'Standardabweichung des Fehlers an Pfad-Punkten:'
    disp(std(normcollector));
    disp 'maximaler Fehler an Pfad-Punkten:'
    disp(max(normcollector));
    disp 'minimaler Fehler an Pfad-Punkten:'
    disp(min(normcollector));
end

%% more plot
if strcmp(verbosity, 'vEyecandy')
% check the correct direction with 3D arrows
figure(pathfig)
hold on
arrow3(emPointsFirstSensor_by_OT',emPointsFirstSensor', 'k', 0.2, 0.2, [], .6)
hold off


figure(pathfig)

xlabel('x')
ylabel('y')
zlabel('z')

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')

if strcmp(verbosity, 'vEyecandy')
    camlight('headlight');
end

plotAuroraVolume(pathfig)
plotAuroraTable(pathfig)
axis image vis3d
view(3)
end
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

minimumErrorRateArrows = 0.90;

distortionNorm = sqrt(Ui.^2+Vi.^2+Wi.^2);

if (strcmp(verbosity, 'vDebug') || strcmp(verbosity, 'vEyecandy'))
vectorfig = figure;
indicesHighDistortion = find(distortionNorm > quantile(distortionNorm(:),minimumErrorRateArrows));
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
view(3)

slicefig = figure;

hold on
h = slice(Xi, Yi, Zi, distortionNorm,[],[],minz:10:maxz);

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
axis image vis3d
view(3)
end

disp 'durchschnittlicher interpolierter fehler'
UVE_Len_notNan = distortionNorm(isfinite(distortionNorm));
UVE_Len_notNan = UVE_Len_notNan(:);
mean(UVE_Len_notNan)

end