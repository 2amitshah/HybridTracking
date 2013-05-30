%% LOADER and parser
close all, clear all;
clc;
%test_measures_EMT_Axes
%05.08 Measurements
%file_path = 'C:\Users\DCUser_02\Desktop\Tracking Calibration\05.08 Measurements\';
file_path = 'C:\Users\AlexSchoch\Desktop\Tracking Calibration\05.17 Measurements\';
file_prefixOT = 'OpticalTracking';
file_prefixEMT = 'EMTracking';
dOT = dir([file_path file_prefixOT '*']);
dEM = dir([file_path file_prefixEMT '*']);

numFiles = numel(dOT);
namesOT = sort({dOT(:).name});
namesEM = sort({dEM(:).name});
    
%dataOT = cell(1, numel(dOT));
%dataEM = cell(1, numel(dEM));
delimiter = ' ';
formatSpecOT = '%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
formatSpecEM = '%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

numPointsOT = 0;
numPointsEM = 0;
for j = 1:numFiles %equals amount of OT-files (each file represents several measurements of one point, in 04.23 set 2 we have 6 points)
    % LOAD OT
    fileIDOT = fopen([file_path namesOT{j}],'r');
    dataArrayOT = textscan(fileIDOT, formatSpecOT, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition = [str2double(dataArrayOT{1,2}), str2double(dataArrayOT{1,3}), str2double(dataArrayOT{1,4})];
    TempOrient = [str2double(dataArrayOT{1,6}), str2double(dataArrayOT{1,7}), str2double(dataArrayOT{1,8}), str2double(dataArrayOT{1,9})];
    TimeStamp = str2double(dataArrayOT{1,11});
    % Stores OT in the cell
    numPointsOT = size(TempPosition)
    for k = 1:numPointsOT
      dataOT{k}.Position=TempPosition(k,:);
      dataOT{k}.Orientation=TempOrient(k,:);
      dataOT{k}.TimeStamp=TimeStamp(k);
    end
    
   % clear TempPosition dataArrayOT TempOrient fileIDOT;
   
    % LOAD EM
    fileIDEM = fopen([file_path namesEM{j}],'r');
    dataArrayEM = textscan(fileIDEM, formatSpecEM, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
    TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
    % Split the raw data according to the 3 sensors
    TimeStamp = str2double(dataArrayEM{1,13});
    index0 = 1; index1 = 1; index2 = 1;
    for i = 1:size(TempPosition)
        switch str2double(dataArrayEM{1,2}(i));
            case 0
                TempPositionFirstSensor(index0,:) = TempPosition(i,:);
                TempOrientFirstSensor(index0,:) = TempOrient(i,:);
                TempTimeFirstSensor(index0) = TimeStamp(i);
                index0 = index0 + 1;
            case 1
                TempPositionSecondSensor(index1,:) = TempPosition(i,:);
                TempOrientSecondSensor(index1,:) = TempOrient(i,:);
                TempTimeSecondSensor(index1) = TimeStamp(i);
                index1 = index1 + 1;
            otherwise
                TempPositionThirdSensor(index2,:) = TempPosition(i,:);
                TempOrientThirdSensor(index2,:) = TempOrient(i,:);
                TempTimeThirdSensor(index2) = TimeStamp(i);
                index2 = index2 + 1;
        end
    end
       
    % Stores EM in the cell
    numPointsEM = size(TempPositionSecondSensor)
    for k = 1:numPointsEM
      dataEM{k}.FirstSensor.Position=TempPositionFirstSensor(k,:);
      dataEM{k}.FirstSensor.Orientation=TempOrientFirstSensor(k,:);
      dataEM{k}.FirstSensor.TimeStamp=TempTimeFirstSensor(k);
      dataEM{k}.SecondSensor.Position=TempPositionSecondSensor(k,:);
      dataEM{k}.SecondSensor.Orientation=TempOrientSecondSensor(k,:);
      dataEM{k}.SecondSensor.TimeStamp=TempTimeSecondSensor(k);
      dataEM{k}.ThirdSensor.Position = [0 0 0];
      dataEM{k}.ThirdSensor.Orientation = [0 0 0 0];
      dataEM{k}.ThirdSensor.TimeStamp=0;
    end

%we just have 2 sensors    

   % clear TempPositionFirstSensor TempOrientFirstSensor TempPositionSecondSensor TempOrientSecondSensor TempPositionThirdSensor TempOrientThirdSensor TempPosition TempOrient dataArrayEM FirstSensor minimum fileIDEM;
end % Loader

%clear dEM dOT delimiter file_path file_prefixEMT file_prefixOT formatSpecOT formatSpecEM j namesEM namesOT dataEMTemp2P dataEMTemp2O index0 index1 index2;





%% Cells to Point-arrays
% Saves the computed points in matrices
%usually
if(numPointsOT < numPointsEM)
    numPoints = numPointsOT;
else
    numPoints = numPointsEM;
end

%plot measurements on time axis
for k = 1:numPoints
  plot(dataOT{k}.TimeStamp, 'bx');
  plot(dataEM{k}.FirstSensor.TimeStamp, 'rx');
  plot(dataEM{k}.SecondSensor.TimeStamp, 'gx');
  hold on
end

goodPoints = 1:numPoints;
%discard the other measurements
% goodPoints = [1 2 6 10];
numPoints = length(goodPoints);

opticalPoints = zeros(3, numPoints);
emPointsFirstSensor = zeros(3,numPoints);
emPointsSecondSensor = zeros(3,numPoints);
emPointsThirdSensor = zeros(3,numPoints);

opticalOrientations = zeros(4, numPoints);
emOrientationsFirstSensor = zeros(4,numPoints);
emOrientationsSecondSensor = zeros(4,numPoints);
emOrientationsThirdSensor = zeros(4,numPoints);
for i = 1:numPoints
    opticalPoints(:,i) = dataOT{goodPoints(i)}.Position;
    emPointsFirstSensor(:,i) = dataEM{goodPoints(i)}.FirstSensor.Position;
% emPointsFirstSensor(:,i) = dataEM{goodPoints(i)}.SecondSensor.Position;
    emPointsSecondSensor(:,i) = dataEM{goodPoints(i)}.SecondSensor.Position;
    emPointsThirdSensor(:,i) = dataEM{goodPoints(i)}.ThirdSensor.Position;
%test output
    norm(emPointsSecondSensor(:,i)-emPointsFirstSensor(:,i))

    opticalOrientations(:,i) = (dataOT{goodPoints(i)}.Orientation)';
    emOrientationsFirstSensor(:,i) = (dataEM{goodPoints(i)}.FirstSensor.Orientation)';
% emOrientationsFirstSensor(:,i) = (dataEM{goodPoints(i)}.SecondSensor.Orientation)';
    emOrientationsSecondSensor(:,i) = (dataEM{goodPoints(i)}.SecondSensor.Orientation)';
    emOrientationsThirdSensor(:,i) = (dataEM{goodPoints(i)}.ThirdSensor.Orientation)';
end
clear i;

%% calculate rotation matrices column wise. Each column represents an axis.
%set length to 5 centimeters
% xaxes_optical = quatrotate(opticalOrientations',[1 0 0]);%numPoints x 3 format
% yaxes_optical = quatrotate(opticalOrientations',[0 1 0]);
% zaxes_optical = quatrotate(opticalOrientations',[0 0 1]);

%################## THIS WAY IT ALSO WORKS
% But the initial solution with unit quaternions (which are not provided (?))
% also works ^^
%         opt_Rs=quat2dcm(opticalOrientations');
%         emFirst_Rs=quat2dcm(emOrientationsFirstSensor');
%         emSecond_Rs=quat2dcm(emOrientationsSecondSensor');
        for i=1:numPoints
            tmp = quat2rot(opticalOrientations(2:4,i));
            opt_Rs(:,:,i) = tmp(1:3,1:3);
            tmp = quat2rot(emOrientationsFirstSensor(2:4,i));
            emFirst_Rs(:,:,i) = tmp(1:3,1:3);
            tmp = quat2rot(emOrientationsSecondSensor(2:4,i));
            emSecond_Rs(:,:,i) = tmp(1:3,1:3);
        end
%##################
xaxes_optical=zeros(numPoints,3);
yaxes_optical=zeros(numPoints,3);
zaxes_optical=zeros(numPoints,3);

xaxes_emFirst=zeros(numPoints,3);
yaxes_emFirst=zeros(numPoints,3);
zaxes_emFirst=zeros(numPoints,3);

xaxes_emSecond=zeros(numPoints,3);
yaxes_emSecond=zeros(numPoints,3);
zaxes_emSecond=zeros(numPoints,3);

for i=1:numPoints
    tmp1(:,:) = opt_Rs(1:3,1,i);
    tmp2(:,:) = opt_Rs(1:3,2,i);
    tmp3(:,:) = opt_Rs(1:3,3,i);

    xaxes_optical(i,:) = tmp1';
    yaxes_optical(i,:) = tmp2';
    zaxes_optical(i,:) = tmp3';
    
    tmp1(:,:) = emFirst_Rs(1:3,1,i);
    tmp2(:,:) = emFirst_Rs(1:3,2,i);
    tmp3(:,:) = emFirst_Rs(1:3,3,i);
    
    xaxes_emFirst(i,:) = tmp1';
    yaxes_emFirst(i,:) = tmp2';
    zaxes_emFirst(i,:) = tmp3';
    
    tmp1(:,:) = emSecond_Rs(1:3,1,i);
    tmp2(:,:) = emSecond_Rs(1:3,2,i);
    tmp3(:,:) = emSecond_Rs(1:3,3,i);
    
    xaxes_emSecond(i,:) = tmp1';
    yaxes_emSecond(i,:) = tmp2';
    zaxes_emSecond(i,:) = tmp3';
end

%calculate rotation matrices column wise. Each column represents an axis.
%set length to 5 centimeters
% xaxes_emFirst = quatrotate(emOrientationsFirstSensor',[1 0 0]);%numPointsx3 format
% yaxes_emFirst = quatrotate(emOrientationsFirstSensor',[0 1 0]);
% zaxes_emFirst = quatrotate(emOrientationsFirstSensor',[0 0 1]);

%calculate rotation matrices column wise. Each column represents an axis.
%set length to 5 centimeters
% xaxes_emSecond = quatrotate(emOrientationsSecondSensor',[1 0 0]);%numPointsx3 format
% yaxes_emSecond = quatrotate(emOrientationsSecondSensor',[0 1 0]);
% zaxes_emSecond = quatrotate(emOrientationsSecondSensor',[0 0 1]);

last_rot_diff=zeros(3);
for i=1:numPoints
    Rot_emFirst = [xaxes_emFirst(i,:)' yaxes_emFirst(i,:)' zaxes_emFirst(i,:)'];
    Rot_emSecond = [xaxes_emSecond(i,:)' yaxes_emSecond(i,:)' zaxes_emSecond(i,:)'];
    
    Rot_difference =   inv(Rot_emFirst)*Rot_emSecond;
    
    [out]=rodrigues(Rot_difference);
    out/norm(out)
    
%     max_element_dist_of_two_rot_matrices = max(max(abs(Rot_difference-last_rot_diff)))
%     last_rot_diff = Rot_difference;
%     
%     [V,D]=eig(Rot_difference)
%     [row,col]=find((real(D) > 0.95)&(real(D) < 1.05));
%     disp('Vector and rotation angle:')
%     rotationvector=V(:,col)'
figure(3)
hold on
line([0;out(1)],[0;out(2)],[0;out(3)], 'LineWidth',3,'Color', [0 0 0]);
plot3(out(1),out(2),out(3),'rx')
hold off
end
xlabel('x')
ylabel('y')
zlabel('z')
axis vis3d image

%% PLOT

c = colormap('lines');

% plot all OT points
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

%% plot all EMT points
figure(2)
plot3(emPointsFirstSensor(1,:), emPointsFirstSensor(2,:),emPointsFirstSensor(3,:), 'x', 'Color', c(1,:) );
hold on
%figure(3)
plot3(emPointsSecondSensor(1,:), emPointsSecondSensor(2,:),emPointsSecondSensor(3,:), 'o', 'Color', c(1,:) );
%hold on
%figure(4)
% plot3(emPointsThirdSensor(1,:), emPointsThirdSensor(2,:),emPointsThirdSensor(3,:), '+', 'Color', c(1,:) );
hold off

title({'EM Tracker position in electromagnetical coordinate system ECS',...
    'orientation is shown in XYZ = RGB',...
    'Red x shows corresponding OT position, black line connects corresponding pairs'})%;'X and Z axis are reversed, so it looks like in our setup'})
xlabel('x')
ylabel('y')
zlabel('z')

% plot all EMT orientations of EM tracker 1

%x axis is red, y axis is green, z axis is blue
hold on
line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+20*xaxes_emFirst(:,1)'],...
    [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+20*xaxes_emFirst(:,2)'],...
    [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+20*xaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [1 0 0]);
hold off
hold on
line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+20*yaxes_emFirst(:,1)'],...
    [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+20*yaxes_emFirst(:,2)'],...
    [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+20*yaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [0 1 0]);
hold off
hold on
line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+20*zaxes_emFirst(:,1)'],...
    [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+20*zaxes_emFirst(:,2)'],...
    [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+20*zaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [0 0 1]);
hold off

% plot all EMT orientations of EM tracker 2

%x axis is red, y axis is green, z axis is blue
hold on
line([emPointsSecondSensor(1,:); emPointsSecondSensor(1,:)+20*xaxes_emSecond(:,1)'],...
    [emPointsSecondSensor(2,:); emPointsSecondSensor(2,:)+20*xaxes_emSecond(:,2)'],...
    [emPointsSecondSensor(3,:); emPointsSecondSensor(3,:)+20*xaxes_emSecond(:,3)'], 'LineWidth',3,'Color', [1 0 1]);%pink
hold off
hold on
line([emPointsSecondSensor(1,:); emPointsSecondSensor(1,:)+20*yaxes_emSecond(:,1)'],...
    [emPointsSecondSensor(2,:); emPointsSecondSensor(2,:)+20*yaxes_emSecond(:,2)'],...
    [emPointsSecondSensor(3,:); emPointsSecondSensor(3,:)+20*yaxes_emSecond(:,3)'], 'LineWidth',3,'Color', [1 1 0]);%yellow
hold off
hold on
line([emPointsSecondSensor(1,:); emPointsSecondSensor(1,:)+20*zaxes_emSecond(:,1)'],...
    [emPointsSecondSensor(2,:); emPointsSecondSensor(2,:)+20*zaxes_emSecond(:,2)'],...
    [emPointsSecondSensor(3,:); emPointsSecondSensor(3,:)+20*zaxes_emSecond(:,3)'], 'LineWidth',3,'Color', [0 1 1]);
hold off

%connect EM Trackers to check for correct respective orientation
line([emPointsFirstSensor(1,:); emPointsSecondSensor(1,:)],[emPointsFirstSensor(2,:); emPointsSecondSensor(2,:)],[emPointsFirstSensor(3,:); emPointsSecondSensor(3,:)])

axis image vis3d
clear c;

%% We will use TSAIleastSquareCalibration.m, prepare matrices:
%Robot is TSAIs WCS (world coordinate system). This is our EM-CS.
%The algorithm needs EMT to EM-CS-Origin homogenuous transformation.

%we start with the first EM tracker
H_OT_to_OCS = zeros(4,4,numPoints);
H_firstEMCS_to_EMT = zeros(4,4,numPoints);
H_firstEMT_to_EMCS = zeros(4,4,numPoints);

for i=1:numPoints
    Rot_emFirst = [xaxes_emFirst(i,:)' yaxes_emFirst(i,:)' zaxes_emFirst(i,:)'];
    H_firstEMT_to_EMCS(:,:,i) = [Rot_emFirst emPointsFirstSensor(:,i);...
                                    0       0       0       1];
    H_firstEMCS_to_EMT(:,:,i) = inv([Rot_emFirst emPointsFirstSensor(:,i);...
                                    0       0       0       1]);
end

%then we calculate the Homogenuous matrices for the Optical tracker
%For Tsai this is grid to cam.
%Cam is OT for us. Grid is the OCS. we need to invert OT coordinates and
%orientation to get the correct transformation direction.
for i=1:numPoints
    Rot_optical = [xaxes_optical(i,:)' yaxes_optical(i,:)' zaxes_optical(i,:)'];
    H_OT_to_OCS(:,:,i) = [Rot_optical opticalPoints(:,i);...
                            0   0   0   1];
end

%% Perform Calibration
disp 'X1: transformation from OT to EMT1'

[Hcam2marker_, err] = TSAIleastSquareCalibration(H_OT_to_OCS, H_firstEMCS_to_EMT)
% we get EMT to OT here (~Hcam2marker_)
% so lets turn it around
X_1 = inv(Hcam2marker_);

[X_1_Laza, err_Laza] = handEyeLaza(H_OT_to_OCS, H_firstEMT_to_EMCS)
% we get EMT to OT here (~X_1_Laza)
% so lets turn it around
X_1_Laza=inv(X_1_Laza);

%well that  didnt work so nice... lets try to do it the other way around

%% trying to plot OT inside the EM-CS
opticalPoints_EMCS=zeros(4,numPoints);
for i=1:numPoints
    temp=H_firstEMT_to_EMCS(:,:,i);
    opticalPoints_EMCS(:,i) = temp * (X_1_Laza * [0;0;0;1]);
end
opticalPoints_EMCS = opticalPoints_EMCS(1:3,:);
figure(2)
hold on
plot3(opticalPoints_EMCS(1,:), opticalPoints_EMCS(2,:), opticalPoints_EMCS(3,:), 'rx', 'MarkerSize', 5)
hold off

%show which points should correlate
hold on
line([opticalPoints_EMCS(1,:); emPointsFirstSensor(1,:)],...
    [opticalPoints_EMCS(2,:); emPointsFirstSensor(2,:)],...
    [opticalPoints_EMCS(3,:); emPointsFirstSensor(3,:)], 'LineWidth',1,'Color', [0 0 0]);
hold off
title('Now this actually should be right')
xlabel('x')
ylabel('y')
zlabel('z')
axis image vis3d


%% some distance statistics
 lol=opticalPoints_EMCS-emPointsFirstSensor;
for i=1:numPoints
norm(lol(:,i))
end
