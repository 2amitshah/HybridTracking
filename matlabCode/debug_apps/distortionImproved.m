function distortion_field = distortionImproved(path, H_OT_to_EMT)

% Change when we move from debug to apps!!!!!!
% Change with the new values
 if ~exist('path','var')
     pathGeneral = fileparts(fileparts(fileparts(which(mfilename))));
     path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];
 end
 
 if ~exist('H_OT_to_EMT', 'var')
    load(which('H_OT_to_EMT.mat'));
 end
 
%% get Y
Y = polaris_to_aurora(path, H_OT_to_EMT);
 
%% get positions
close all;

testrow_name_EMT = 'distorEMT';
testrow_name_OT = 'distorOT';

% get data for hand/eye calib
[data_EMTraw, data_OTraw,~,~] = read_TrackingFusion_files(path);

H_EMT_to_EMCSrawcell = trackingdata_to_matrices(data_EMTraw,'CppCodeQuat');
H_OT_to_OCSrawcell = trackingdata_to_matrices(data_OTraw,'CppCodeQuat');

EMThandle = Plot_points(H_EMT_to_EMCSrawcell)

%prepare data
numPts = size(data_EMT,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

for i = 1:numPts

    %insert rotation into homogeneous matrix
    mat{1}(:,:,i) = quat2rot((data_EMT{i,1}.orientation(2:4))');

    mat{2}(:,:,i) = quat2rot((data_OT{i}.orientation(2:4))');

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

% plot all OT_in_EMCS positions

r_sphere = 4;
[Xsp, Ysp, Zsp] = sphere;
Xsp = Xsp * r_sphere;
Ysp = Ysp * r_sphere;
Zsp = Zsp * r_sphere;

figure(1)
hold on
plot3(opticalPoints_in_EMCS(1,:), opticalPoints_in_EMCS(2,:), opticalPoints_in_EMCS(3,:), 'x', 'Color', c(1,:) );
hold off
for i = 1:numPts
    % plot little sphere in grey to look like the OT
    hold on
    surf(Xsp+opticalPoints_in_EMCS(1,i), Ysp+opticalPoints_in_EMCS(2,i), Zsp+opticalPoints_in_EMCS(3,i), 'EdgeColor', 'none', 'FaceColor', [.9 .9 .9], 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
    hold off
end
title({'Optical Center position in optical coordinate system OCS',...
    'orientation is shown in XYZ = RGB'})
xlabel('x')
ylabel('y')
zlabel('z')

% plot all OT orientations
%x axis is red, y axis is green, z axis is blue
hold on
line([opticalPoints_in_EMCS(1,:); opticalPoints_in_EMCS(1,:)+30*xaxes_optical_in_EMCS(:,1)'],...
    [opticalPoints_in_EMCS(2,:); opticalPoints_in_EMCS(2,:)+30*xaxes_optical_in_EMCS(:,2)'],...
    [opticalPoints_in_EMCS(3,:); opticalPoints_in_EMCS(3,:)+30*xaxes_optical_in_EMCS(:,3)'], 'LineWidth',3,'Color', [1 0 0]);
hold off
hold on
line([opticalPoints_in_EMCS(1,:); opticalPoints_in_EMCS(1,:)+30*yaxes_optical_in_EMCS(:,1)'],...
    [opticalPoints_in_EMCS(2,:); opticalPoints_in_EMCS(2,:)+30*yaxes_optical_in_EMCS(:,2)'],...
    [opticalPoints_in_EMCS(3,:); opticalPoints_in_EMCS(3,:)+30*yaxes_optical_in_EMCS(:,3)'], 'LineWidth',3,'Color', [0 1 0]);
hold off
hold on
line([opticalPoints_in_EMCS(1,:); opticalPoints_in_EMCS(1,:)+30*zaxes_optical_in_EMCS(:,1)'],...
    [opticalPoints_in_EMCS(2,:); opticalPoints_in_EMCS(2,:)+30*zaxes_optical_in_EMCS(:,2)'],...
    [opticalPoints_in_EMCS(3,:); opticalPoints_in_EMCS(3,:)+30*zaxes_optical_in_EMCS(:,3)'], 'LineWidth',3,'Color', [0 0 1]);
hold off

% plot all EMT positions
figure(1)
hold on
plot3(emPointsFirstSensor(1,:), emPointsFirstSensor(2,:),emPointsFirstSensor(3,:), 'x', 'Color', c(1,:) );
hold off
title({'EM Tracker position in electromagnetical coordinate system (EMCS)',...
    'orientation is shown in XYZ = RGB',...
    'transformed OT position in EMCS is shown as red x, corresponding positions are connected'})
xlabel('x')
ylabel('y')
zlabel('z')

% plot all EMT orientations of EM tracker 1

hold on
line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+30*xaxes_emFirst(:,1)'],...
    [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+30*xaxes_emFirst(:,2)'],...
    [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+30*xaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [1 0 0]);
hold off
hold on
line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+30*yaxes_emFirst(:,1)'],...
    [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+30*yaxes_emFirst(:,2)'],...
    [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+30*yaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [0 1 0]);
hold off
hold on
line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+30*zaxes_emFirst(:,1)'],...
    [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+30*zaxes_emFirst(:,2)'],...
    [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+30*zaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [0 0 1]);
hold off

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')

axis image vis3d

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
% plot all EMT positions
figure(1)
hold on
plot3(emPointsFirstSensor_by_OT(1,:), emPointsFirstSensor_by_OT(2,:),emPointsFirstSensor_by_OT(3,:), 'x', 'Color', c(1,:), 'MarkerSize', 10 );
hold off


% plot all EMT orientations of EM tracker 1

hold on
line([emPointsFirstSensor_by_OT(1,:); emPointsFirstSensor_by_OT(1,:)+30*xaxes_emFirst_by_OT(:,1)'],...
    [emPointsFirstSensor_by_OT(2,:); emPointsFirstSensor_by_OT(2,:)+30*xaxes_emFirst_by_OT(:,2)'],...
    [emPointsFirstSensor_by_OT(3,:); emPointsFirstSensor_by_OT(3,:)+30*xaxes_emFirst_by_OT(:,3)'], 'LineWidth',3,'Color', [1 1 0]);
hold off
hold on
line([emPointsFirstSensor_by_OT(1,:); emPointsFirstSensor_by_OT(1,:)+30*yaxes_emFirst_by_OT(:,1)'],...
    [emPointsFirstSensor_by_OT(2,:); emPointsFirstSensor_by_OT(2,:)+30*yaxes_emFirst_by_OT(:,2)'],...
    [emPointsFirstSensor_by_OT(3,:); emPointsFirstSensor_by_OT(3,:)+30*yaxes_emFirst_by_OT(:,3)'], 'LineWidth',3,'Color', [1 0 1]);
hold off
hold on
line([emPointsFirstSensor_by_OT(1,:); emPointsFirstSensor_by_OT(1,:)+30*zaxes_emFirst_by_OT(:,1)'],...
    [emPointsFirstSensor_by_OT(2,:); emPointsFirstSensor_by_OT(2,:)+30*zaxes_emFirst_by_OT(:,2)'],...
    [emPointsFirstSensor_by_OT(3,:); emPointsFirstSensor_by_OT(3,:)+30*zaxes_emFirst_by_OT(:,3)'], 'LineWidth',3,'Color', [0 1 1]);
hold off

%% distance output
H_diff_EMT = zeros(4,4,numPts);
for i = 1:numPts
    H_diff_EMT(:,:,i) = inv(H_EMT_to_EMCS(:,:,i))*H_EMT_to_EMCS_by_OT(:,:,i);
    disp 'deviation of EMT due to field errors'
    norm(H_diff_EMT(1:3,4,i))
end

%% check the correct direction
% try TriScatteredInterp
% use arrow3
% for i = 1:numPts
    hold on
    arrow3(emPointsFirstSensor_by_OT',emPointsFirstSensor', 'k', 1, 1, [], .6)
    hold off
% end


%% interpolate vector field

Fu = scatteredInterpolant(emPointsFirstSensor_by_OT', permute(H_diff_EMT(1,4,:),[3 2 1]), 'natural', 'none');
Fv = scatteredInterpolant(emPointsFirstSensor_by_OT', permute(H_diff_EMT(2,4,:),[3 2 1]), 'natural', 'none');
Fw = scatteredInterpolant(emPointsFirstSensor_by_OT', permute(H_diff_EMT(3,4,:),[3 2 1]), 'natural', 'none');

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
mean(UVE_Len_notNan)
end