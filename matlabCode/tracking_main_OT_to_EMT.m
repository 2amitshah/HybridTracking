%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to one EMT and one OT dataset,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT.
%calibration is done for Optical to the first EM tracker.
%%%%%%%%%%%%%
%% data read in
% do preparation
% clear variables globals;
close all;

path = '..\measurements\testmfrom_NDItrack';
testrow_name_EMT = 'hybridEMT';
testrow_name_OT = 'hybridOT';

% get data for hand/eye calib
[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
[data_OT] = read_NDI_tracking_files(path, testrow_name_OT);

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
    Hgrid2cam(:,:,i)        = inv(mat{1}(:,:,i)); %(=EMCS_to_EMT)
    H_EMT_to_EMCS(:,:,i)    = mat{1}(:,:,i);
    Hmarker2world(:,:,i)    = mat{2}(:,:,i);      %(=OT_to_OCS)
end

%% plot position data
% plot to see how positions were recorded
for i = 1:numPts
    %optical
    %rotation
    xaxes_optical(i,:) = Hmarker2world(1:3,1,i)';
    yaxes_optical(i,:) = Hmarker2world(1:3,2,i)';
    zaxes_optical(i,:) = Hmarker2world(1:3,3,i)';
    %translation
    opticalPoints(:,i) = Hmarker2world(1:3,4,i);
    
    %em tracker
    %rotation
    xaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,1,i)';
    yaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,2,i)';
    zaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,3,i)';
    %translation
    emPointsFirstSensor(:,i) = H_EMT_to_EMCS(1:3,4,i);
end
c = colormap('lines');

% plot all OT positions
figure(1)
hold on
plot3(opticalPoints(1,:), opticalPoints(2,:), opticalPoints(3,:), 'x', 'Color', c(1,:) );
hold off
title({'Optical Center position in optical coordinate system OCS',...
    'orientation is shown in XYZ = RGB'})
xlabel('x')
ylabel('y')
zlabel('z')

% plot all OT orientations
%x axis is red, y axis is green, z axis is blue
hold on
line([opticalPoints(1,:); opticalPoints(1,:)+30*xaxes_optical(:,1)'],...
    [opticalPoints(2,:); opticalPoints(2,:)+30*xaxes_optical(:,2)'],...
    [opticalPoints(3,:); opticalPoints(3,:)+30*xaxes_optical(:,3)'], 'LineWidth',3,'Color', [1 0 0]);
hold off
hold on
line([opticalPoints(1,:); opticalPoints(1,:)+30*yaxes_optical(:,1)'],...
    [opticalPoints(2,:); opticalPoints(2,:)+30*yaxes_optical(:,2)'],...
    [opticalPoints(3,:); opticalPoints(3,:)+30*yaxes_optical(:,3)'], 'LineWidth',3,'Color', [0 1 0]);
hold off
hold on
line([opticalPoints(1,:); opticalPoints(1,:)+30*zaxes_optical(:,1)'],...
    [opticalPoints(2,:); opticalPoints(2,:)+30*zaxes_optical(:,2)'],...
    [opticalPoints(3,:); opticalPoints(3,:)+30*zaxes_optical(:,3)'], 'LineWidth',3,'Color', [0 0 1]);
hold off
axis image vis3d

% plot all EMT positions
figure(2)
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

%% calibration
%Sorting the positions

% [X_1_Laza, err_Laza] = handEyeLaza(Hmarker2world, H_EMT_to_EMCS)
% we get EMT to OT here (~X_1_Laza)
% so lets turn it around
% X_1_Laza=inv(X_1_Laza);

% index = sortHandEyeMovement(Hmarker2world);
% Hmarker2world = Hmarker2world(:,:,index);
% Hgrid2cam = Hgrid2cam(:,:,index);
% H_EMT_to_EMCS = H_EMT_to_EMCS(:,:,index);

[Hcam2marker_, err] = TSAIleastSquareCalibration(Hmarker2world, Hgrid2cam)
% we get EMT to OT here (~Hcam2marker_)
% so lets turn it around
X_1 = inv(Hcam2marker_);
disp 'Distance according to TSAI algorithm:'
disp(norm(X_1(1:3,4)))

[X_1_Laza, err_Laza, goodCombinations] = handEyeLaza_goodcombinations(Hmarker2world, H_EMT_to_EMCS)
% we get EMT to OT here (~X_1_Laza)
% so lets turn it around
X_1_Laza=inv(X_1_Laza);

% [X_1_dual, err_dual] = hand_eye_dual_quaternion(Hmarker2world, H_EMT_to_EMCS)

% [X_1_navy, err_navy] = navy_calibration(Hmarker2world, H_EMT_to_EMCS)

% [X_1_inria, err_inria] = inria_calibration(Hmarker2world, H_EMT_to_EMCS)

%% plot OT inside the EM-CS

%plot the transformation of OT in OCS into OT in EMCS
opticalPoints_EMCS_transl = zeros(4,numPts);

for i=1:numPts
    temp_EMT_to_EMCS=H_EMT_to_EMCS(:,:,i);
    opticalPoints_EMCS_transl(:,i) = temp_EMT_to_EMCS * (X_1_Laza * [0;0;0;1]);
end

opticalPoints_EMCS=opticalPoints_EMCS_transl(1:3,:);

figure(2)
hold on
plot3(opticalPoints_EMCS(1,:), opticalPoints_EMCS(2,:), opticalPoints_EMCS(3,:), 'rx', 'MarkerSize', 5)
hold off

% show which OT and EMT should correlate
hold on
line([opticalPoints_EMCS(1,:); emPointsFirstSensor(1,:)],...
    [opticalPoints_EMCS(2,:); emPointsFirstSensor(2,:)],...
    [opticalPoints_EMCS(3,:); emPointsFirstSensor(3,:)], 'LineWidth',1,'Color', [0 0 0]);
hold off

%make the plot nicer
%plot cylinder which looks like our tool
r_cylinder = 26.5/2;
[Xcy,Ycy,Zcy] = cylinder(r_cylinder,20);

r_sphere = 4;
[Xsp, Ysp, Zsp] = sphere;
Xsp = Xsp * r_sphere;
Ysp = Ysp * r_sphere;
Zsp = Zsp * r_sphere;

H_EMT_to_EMCS_tmp = H_EMT_to_EMCS;

for i=1:numPts

    %remove offset
    Zcy_temp = Zcy-.5;
    %stretch to correct length
    Zcy_temp = Zcy_temp*100; %10cm
    
    %rearrange EMT local cordinate system, so z points along the tool and
    %not orthogonal to it
    H_EMT_to_EMCS_tmp(:,:,i)=H_EMT_to_EMCS(:,[2 3 1 4],i);
    
    Xcy_temp = H_EMT_to_EMCS_tmp(1,1,i)*Xcy + H_EMT_to_EMCS_tmp(1,2,i)*Ycy + H_EMT_to_EMCS_tmp(1,3,i)*Zcy_temp;
    Ycy_temp = H_EMT_to_EMCS_tmp(2,1,i)*Xcy + H_EMT_to_EMCS_tmp(2,2,i)*Ycy + H_EMT_to_EMCS_tmp(2,3,i)*Zcy_temp;
    Zcy_temp = H_EMT_to_EMCS_tmp(3,1,i)*Xcy + H_EMT_to_EMCS_tmp(3,2,i)*Ycy + H_EMT_to_EMCS_tmp(3,3,i)*Zcy_temp;    
       
    %shift cylinder along axes to be positioned like our tool
    %center at EMT position
    Xcy_temp = Xcy_temp + emPointsFirstSensor(1,i);
    Ycy_temp = Ycy_temp + emPointsFirstSensor(2,i);
    Zcy_temp = Zcy_temp + emPointsFirstSensor(3,i);
    %shift along radius, plus 3mm offset, since tool looks like that
    Xcy_temp = Xcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(1,2,i);
    Ycy_temp = Ycy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(2,2,i);
    Zcy_temp = Zcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(3,2,i);
    %shift along new z axis to represent position of EMT on tool correctly
    Xcy_temp = Xcy_temp + 30*H_EMT_to_EMCS_tmp(1,3,i);
    Ycy_temp = Ycy_temp + 30*H_EMT_to_EMCS_tmp(2,3,i);
    Zcy_temp = Zcy_temp + 30*H_EMT_to_EMCS_tmp(3,3,i);
    %shift EMT 7mm to the side, since EMT local coordinate system is not
    %defined in the center of the tracker disc, but at the bottom of one of the
    %screws (the right one)
    Xcy_temp = Xcy_temp - 7*H_EMT_to_EMCS_tmp(1,1,i);
    Ycy_temp = Ycy_temp - 7*H_EMT_to_EMCS_tmp(2,1,i);
    Zcy_temp = Zcy_temp - 7*H_EMT_to_EMCS_tmp(3,1,i);
    
    %plot cylinder
    hold on
    surf(Xcy_temp, Ycy_temp, Zcy_temp, 'EdgeColor', 'none', 'FaceColor', 'y', 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
    hold off
    %plot little sphere in grey to look like the OT
    hold on
    surf(Xsp+opticalPoints_EMCS(1,i), Ysp+opticalPoints_EMCS(2,i), Zsp+opticalPoints_EMCS(3,i), 'EdgeColor', 'none', 'FaceColor', [.9 .9 .9], 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
    hold off
    %plot white stick that connects OT to the tool
    hold on
    line([opticalPoints_EMCS(1,i);opticalPoints_EMCS(1,i)+20*H_EMT_to_EMCS_tmp(1,2,i)],...
        [opticalPoints_EMCS(2,i);opticalPoints_EMCS(2,i)+20*H_EMT_to_EMCS_tmp(2,2,i)],...
        [opticalPoints_EMCS(3,i);opticalPoints_EMCS(3,i)+20*H_EMT_to_EMCS_tmp(3,2,i)], 'Color', 'w', 'LineWidth', 3)
    hold off
end

% Plot lines that connect position-pairs that contributed to the
% calculation
k=0;
goodCombinationsTemp = goodCombinations;
for i = 1:numPts,
    for j = i+1:numPts;
        k=k+1;
        if numel(goodCombinationsTemp~=0)
        if k==goodCombinationsTemp(1)
            goodCombinationsTemp = goodCombinationsTemp(2:end);
            %plot line that connects used positions pairs
            hold on
            line([opticalPoints_EMCS(1,i);opticalPoints_EMCS(1,j)],...
                [opticalPoints_EMCS(2,i);opticalPoints_EMCS(2,j)],...
                [opticalPoints_EMCS(3,i);opticalPoints_EMCS(3,j)], 'Color', 'y', 'LineWidth', 4, 'LineStyle', '--')
            hold off
        end
        end
    end
end


% set axes and lighting
xlabel('x')
ylabel('y')
zlabel('z')
axis image vis3d

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')

camlight('headlight')
lighting gouraud

%% some distance statistics
 diff=opticalPoints_EMCS-emPointsFirstSensor;
 disp 'Distance from OT to EMT in all positions:'
 disp(norm(diff(:,1)));



