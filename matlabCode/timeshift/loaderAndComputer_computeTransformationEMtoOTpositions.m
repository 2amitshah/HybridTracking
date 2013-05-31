function [transformationOpticalToFirstSensor]=loaderAndComputer_computeTransformationEMtoOTpositions(file_path)

%% LOADER
%close all, clear all;
%clc;

%file_path = 'C:\Users\AlexSchoch\Desktop\Tracking Calibration\04.23 Measurements\Set 2\';
%file_path = 'C:\Users\AlexSchoch\Desktop\Tracking Calibration\05.23 Measurements\';

file_prefixOT = 'OpticalTracking';
file_prefixEMT = 'EMTracking';
dOT = dir([file_path file_prefixOT '*']);
dEM = dir([file_path file_prefixEMT '*']);

numPoints = numel(dOT);
namesOT = sort({dOT(:).name});
namesEM = sort({dEM(:).name});
    
dataOT = cell(1, numel(dOT));
dataEM = cell(1, numel(dEM));
delimiter = ' ';
formatSpecOT = '%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
formatSpecEM = '%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

for j = 1:numPoints %equals amount of OT-files (each file represents several measurements of one point, in 04.23 set 2 we have 6 points)
    % LOAD OT
    fileIDOT = fopen([file_path namesOT{j}],'r');
    dataArrayOT = textscan(fileIDOT, formatSpecOT, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition = [str2double(dataArrayOT{1,2}), str2double(dataArrayOT{1,3}), str2double(dataArrayOT{1,4})];
    TempOrient = [str2double(dataArrayOT{1,6}), str2double(dataArrayOT{1,7}), str2double(dataArrayOT{1,8}), str2double(dataArrayOT{1,9})];
    % Stores OT in the cell
    dataOT{j}.Position=mean(TempPosition,1);
    dataOT{j}.Orientation=mean(TempOrient,1);
    
    fclose(fileIDOT);
    clear TempPosition dataArrayOT TempOrient fileIDOT;
   
    % LOAD EM
    fileIDEM = fopen([file_path namesEM{j}],'r');
    dataArrayEM = textscan(fileIDEM, formatSpecEM, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
    TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
    % Split the raw data according to the 3 sensors
    index0 = 1; index1 = 1; index2 = 1;
    for i = 1:size(TempPosition)
        switch str2double(dataArrayEM{1,2}(i));
            case 0
                TempPositionFirstSensor(index0,:) = TempPosition(i,:);
                TempOrientFirstSensor(index0,:) = TempOrient(i,:);
                index0 = index0 + 1;
%             case 1
%                 TempPositionSecondSensor(index1,:) = TempPosition(i,:);
%                 TempOrientSecondSensor(index1,:) = TempOrient(i,:);
%                 index1 = index1 + 1;
%             otherwise
%                 TempPositionThirdSensor(index2,:) = TempPosition(i,:);
%                 TempOrientThirdSensor(index2,:) = TempOrient(i,:);
%                 index2 = index2 + 1;
        end
    end
       
    % Stores EM in the cell
    position(1) =0;
    position(2) =0;
    position(3) =0;
    amount = 0;
    for k=1:size(TempPosition)
        if (TempPositionFirstSensor(k,1) < 10000 && TempPositionFirstSensor(k,1) > -10000 && TempPositionFirstSensor(k,2) < 10000 && TempPositionFirstSensor(k,2) > -10000 && TempPositionFirstSensor(k,3) < 10000 && TempPositionFirstSensor(k,3) > -10000)
        position(1) = position(1) + TempPositionFirstSensor(k,1);
        position(2) = position(2) + TempPositionFirstSensor(k,2);
        position(3) = position(3) + TempPositionFirstSensor(k,3);
        amount = amount+1;
        end
    end
    dataEM{j}.FirstSensor.Position(1) = position(1) / amount;
    dataEM{j}.FirstSensor.Position(2) = position(2) / amount;
    dataEM{j}.FirstSensor.Position(3) = position(3) / amount;
    
    %dataEM{j}.FirstSensor.Position = mean(TempPositionFirstSensor,1);
    %dataEM{j}.FirstSensor.Orientation = mean(TempOrientFirstSensor,1);
    
%     dataEM{j}.SecondSensor.Position = mean(TempPositionSecondSensor,1);
%     dataEM{j}.SecondSensor.Orientation = mean(TempOrientSecondSensor,1);
%     dataEM{j}.ThirdSensor.Position = mean(TempPositionThirdSensor,1);
%     dataEM{j}.ThirdSensor.Orientation = mean(TempOrientThirdSensor,1);  

    clear TempPositionFirstSensor TempOrientFirstSensor TempPositionSecondSensor TempOrientSecondSensor TempPositionThirdSensor TempOrientThirdSensor TempPosition TempOrient dataArrayEM FirstSensor minimum fileIDEM;
end % Loader

clear dEM dOT delimiter file_path file_prefixEMT file_prefixOT formatSpecOT formatSpecEM j namesEM namesOT dataEMTemp2P dataEMTemp2O index0 index1 index2;


% %% PLOT
% 
% c = colormap('lines');
% 
% % Initial points
% figure(1)
% plot3(dataOT{1}.Position(1), dataOT{1}.Position(2),dataOT{1}.Position(3), 'x', 'Color', c(1,:) );
% hold on
% figure(2)
% plot3(dataEM{1}.FirstSensor.Position(1), dataEM{1}.FirstSensor.Position(2),dataEM{1}.FirstSensor.Position(3), 'x', 'Color', c(1,:) );
% hold on
% % figure(3)
% % plot3(dataEM{1}.SecondSensor.Position(1), dataEM{1}.SecondSensor.Position(2),dataEM{1}.SecondSensor.Position(3), 'x', 'Color', c(1,:) );
% % hold on
% % figure(4)
% % plot3(dataEM{1}.ThirdSensor.Position(1), dataEM{1}.ThirdSensor.Position(2),dataEM{1}.ThirdSensor.Position(3), 'x', 'Color', c(1,:) );
% % hold on
% 
% % For each point, plot its value on the graph, each plot shows the "way" of
% % the marker (the 6 points in our case)
% for i = 2:numPoints
%     figure(1)
%     plot3([dataOT{i-1}.Position(1) dataOT{i}.Position(1)],...
%         [dataOT{i-1}.Position(2) dataOT{i}.Position(2)],...
%         [dataOT{i-1}.Position(3) dataOT{i}.Position(3)], '-x', 'Color', c(i,:) );
%     hold on
%     figure(2)
%     plot3([dataEM{i-1}.FirstSensor.Position(1) dataEM{i}.FirstSensor.Position(1)],...
%         [dataEM{i-1}.FirstSensor.Position(2) dataEM{i}.FirstSensor.Position(2)],...
%         [dataEM{i-1}.FirstSensor.Position(3) dataEM{i}.FirstSensor.Position(3)], '-x', 'Color', c(i,:) );
%     hold on
% %     figure(3)
% %     plot3([dataEM{i-1}.SecondSensor.Position(1) dataEM{i}.SecondSensor.Position(1)],...
% %         [dataEM{i-1}.SecondSensor.Position(2) dataEM{i}.SecondSensor.Position(2)],...
% %         [dataEM{i-1}.SecondSensor.Position(3) dataEM{i}.SecondSensor.Position(3)], '-x', 'Color', c(i,:) );
% %     hold on
% %     figure(4)
% %     plot3([dataEM{i-1}.ThirdSensor.Position(1) dataEM{i}.ThirdSensor.Position(1)],...
% %         [dataEM{i-1}.ThirdSensor.Position(2) dataEM{i}.ThirdSensor.Position(2)],...
% %         [dataEM{i-1}.ThirdSensor.Position(3) dataEM{i}.ThirdSensor.Position(3)], '-x', 'Color', c(i,:) );
% %     hold on
% %     
% end
% 
% % Computes distance between points and norm. For validation
% opticalDistance(1,:) = dataOT{2}.Position - dataOT{1}.Position;
% opticalDistance(2,:) = dataOT{3}.Position - dataOT{2}.Position;
% opticalDistance(3,:) = dataOT{4}.Position - dataOT{3}.Position;
% normOptical(1)=norm(opticalDistance(1,:));
% normOptical(2)=norm(opticalDistance(2,:));
% normOptical(3)=norm(opticalDistance(3,:));
% 
% clear c;
% 
% %% EM
% % Computes distance between points and norm. For validation. For each
% % sensor!
% for i = 1:numPoints-1
%     opticalDistance(i,:) = dataOT{i+1}.Position - dataOT{i}.Position;
%     emFirstSensorDistance(i,:) = dataEM{i+1}.FirstSensor.Position - dataEM{i}.FirstSensor.Position;
% %     emSecondSensorDistance(i,:) = dataEM{i+1}.SecondSensor.Position - dataEM{i}.SecondSensor.Position;
% %     emThirdSensorDistance(i,:) = dataEM{i+1}.ThirdSensor.Position - dataEM{i}.ThirdSensor.Position;
%     normOptical(i) = norm(opticalDistance(i,:));
%     normEMFirstSensor(i)=norm(emFirstSensorDistance(i,:));
% %     normEMSecondSensor(i)=norm(emSecondSensorDistance(i,:));
% %     normEMThirdSensor(i)=norm(emThirdSensorDistance(i,:));
% end
% 
% %%
% close all
figure
for i = 1:numPoints;
plot3(dataEM{i}.FirstSensor.Position(1), dataEM{i}.FirstSensor.Position(2), dataEM{i}.FirstSensor.Position(3), 'r*');
hold on;
plot3(dataOT{i}.Position(1), dataOT{i}.Position(2), dataOT{i}.Position(3), 'b*')
box on;
grid on;
end


%% Points
% Saves the computed point in matrices
opticalPoints = zeros(3, numPoints);
emPointsFirstSensor = zeros(3,numPoints);
% emPointsSecondSensor = zeros(3,numPoints);
% emPointsThirdSensor = zeros(3,numPoints);
for i = 1:numPoints
    opticalPoints(:,i) = dataOT{i}.Position;
    emPointsFirstSensor(:,i) = dataEM{i}.FirstSensor.Position;
%     emPointsSecondSensor(:,i) = dataEM{i}.SecondSensor.Position;
     % emPointsThirdSensor(:,i) = dataEM{i}.ThirdSensor.Position;
end

clear i;
% Obtains transformation matrix
transformationOpticalToFirstSensor = absor(opticalPoints,emPointsFirstSensor);
% transformationOpticalToSecondSensor = absor(opticalPoints,emPointsSecondSensor);
% transformationOpticalToThirdSensor = absor(opticalPoints,emPointsThirdSensor,'doScale',true);

% norm between the em tracking points should always be the same

% for i = 1:numPoints
%    emFirstSecondTrans{i} = dataEM{i}.FirstSensor.Position - dataEM{i}.SecondSensor.Position;
%    emFirstSecondTransNorms{i} = norm(emFirstSecondTrans{i});
% end
% for i = 1:numPoints
%    emSecondThirdTrans{i} = dataEM{i}.SecondSensor.Position - dataEM{i}.ThirdSensor.Position;
%    emSecondThirdTransNorms{i} = norm(emSecondThirdTrans{i});
% end
% for i = 1:numPoints
%    emFirstThirdTrans{i} = dataEM{i}.FirstSensor.Position - dataEM{i}.ThirdSensor.Position;
%    emFirstThirdTransNorms{i} = norm(emFirstThirdTrans{i});
% end

for i = 1:numPoints
   emFirstOpticalTrans{i} = dataEM{i}.FirstSensor.Position - dataOT{i}.Position;
   emFirstOpticalTransNorms{i} = norm(emFirstOpticalTrans{i});
end

% emFirstSecondTransNorms
% emFirstThirdTransNorms
% emSecondThirdTransNorms
emFirstOpticalTransNorms

dataOTToEMOne = dataOT;

figure
for i = 1:numPoints;
    pointvector = [dataOTToEMOne{i}.Position(1); dataOTToEMOne{i}.Position(2); dataOTToEMOne{i}.Position(3)];
    pointvector = transformationOpticalToFirstSensor.R * pointvector + transformationOpticalToFirstSensor.t;
    dataOTToEMOne{i}.Position(1) = pointvector(1);
    dataOTToEMOne{i}.Position(2) = pointvector(2);
    dataOTToEMOne{i}.Position(3) = pointvector(3);
plot3(dataEM{i}.FirstSensor.Position(1), dataEM{i}.FirstSensor.Position(2), dataEM{i}.FirstSensor.Position(3), 'ro');
hold on;
plot3(dataOTToEMOne{i}.Position(1), dataOTToEMOne{i}.Position(2), dataOTToEMOne{i}.Position(3), 'bo')
box on;
grid on;
end


figure
for i = 1:numPoints;
    pointvector = [dataEM{i}.FirstSensor.Position(1); dataEM{i}.FirstSensor.Position(2); dataEM{i}.FirstSensor.Position(3)];
    pointvector = inv(transformationOpticalToFirstSensor.R) * (pointvector - transformationOpticalToFirstSensor.t);
    dataEM{i}.FirstSensor.Position(1) = pointvector(1);
    dataEM{i}.FirstSensor.Position(2) = pointvector(2);
    dataEM{i}.FirstSensor.Position(3) = pointvector(3);
plot3(dataEM{i}.FirstSensor.Position(1), dataEM{i}.FirstSensor.Position(2), dataEM{i}.FirstSensor.Position(3), 'ro');
hold on;
plot3(dataOT{i}.Position(1), dataOT{i}.Position(2), dataOT{i}.Position(3), 'bo')
box on;
grid on;
end

% figure
% for i = 1:numPoints;
%     pointvector = [dataEM{i}.SecondSensor.Position(1); dataEM{i}.SecondSensor.Position(2); dataEM{i}.SecondSensor.Position(3)];
%     pointvector = inv(transformationOpticalToSecondSensor.R) * (pointvector - transformationOpticalToSecondSensor.t);
%     dataEM{i}.SecondSensor.Position(1) = pointvector(1);
%     dataEM{i}.SecondSensor.Position(2) = pointvector(2);
%     dataEM{i}.SecondSensor.Position(3) = pointvector(3);
% plot3(dataEM{i}.SecondSensor.Position(1), dataEM{i}.SecondSensor.Position(2), dataEM{i}.SecondSensor.Position(3), 'ro');
% hold on;
% plot3(dataOT{i}.Position(1), dataOT{i}.Position(2), dataOT{i}.Position(3), 'bo')
% box on;
% grid on;
% end

% figure
% for i = 1:numPoints;
%     pointvector = [dataEM{i}.ThirdSensor.Position(1); dataEM{i}.ThirdSensor.Position(2); dataEM{i}.ThirdSensor.Position(3)];
%     pointvector = inv(transformationOpticalToThirdSensor.R) * (pointvector - transformationOpticalToThirdSensor.t);
%     dataEM{i}.ThirdSensor.Position(1) = pointvector(1);
%     dataEM{i}.ThirdSensor.Position(2) = pointvector(2);
%     dataEM{i}.ThirdSensor.Position(3) = pointvector(3);
% plot3(dataEM{i}.ThirdSensor.Position(1), dataEM{i}.ThirdSensor.Position(2), dataEM{i}.ThirdSensor.Position(3), 'ro');
% hold on;
% plot3(dataOT{i}.Position(1), dataOT{i}.Position(2), dataOT{i}.Position(3), 'bo')
% box on;
% grid on;
% end

for i = 1:numPoints
    sqrt((dataEM{i}.FirstSensor.Position(1) - dataOT{i}.Position(1))^2 + (dataEM{i}.FirstSensor.Position(2) - dataOT{i}.Position(2))^2 + (dataEM{i}.FirstSensor.Position(3) - dataOT{i}.Position(3))^2)
end
% translations should be slightly different
transformationOpticalToFirstSensor.t
% transformationOpticalToSecondSensor.t
%transformationOpticalToThirdSensor.t

% rotations should be the same..
transformationOpticalToFirstSensor.R
% transformationOpticalToSecondSensor.R
%transformationOpticalToThirdSensor.R

transformationOpticalToFirstSensor.s
% transformationOpticalToSecondSensor.s
%transformationOpticalToThirdSensor.s


% Computation points and diferences with optical values
emPointsFirstSensorComputed = transformationOpticalToFirstSensor.s.*transformationOpticalToFirstSensor.R*opticalPoints + repmat(transformationOpticalToFirstSensor.t,1,size(opticalPoints,2));
emPointsFirstSensorDiferences = abs(emPointsFirstSensorComputed - emPointsFirstSensor);
% emPointsSecondSensorComputed = transformationOpticalToSecondSensor.s.*transformationOpticalToSecondSensor.R*opticalPoints + repmat(transformationOpticalToSecondSensor.t,1,size(opticalPoints,2));
% emPointsSecondSensorDiferences = abs(emPointsSecondSensorComputed - emPointsSecondSensor);
%emPointsThirdSensorComputed = transformationOpticalToThirdSensor.s.*transformationOpticalToThirdSensor.R*opticalPoints + repmat(transformationOpticalToThirdSensor.t,1,size(opticalPoints,2));
%emPointsThirdSensorDiferences = abs(emPointsThirdSensorComputed - emPointsThirdSensor);

end