%this script uses a fixed timeshift (which has been computed using
%loaderAndComputer_computeTimeShift.m) and adds it to the timestamps of the
%points coming from the optical tracking system. This leads to having the
%respective points of the EM tracking system and the OT tracking system
%having the same time stamp when they belong to the same position in space.

%% LOADER and parser
close all, clear all;
clc;
timeshift= -2.5*10^7; %added to EM results
%05.08 Measurements
%file_path = 'C:\Users\DCUser_02\Dropbox\Masterarbeit\HybridTracking_shared\matlabCode\timeshift\05.29 Measurements\';
file_path = 'C:\Users\Nicola\Documents\Studium\SS 13\CP\Tracking Calibration\05.29 Measurements\';
file_prefixOT = 'cont_OpticalTracking_1';
file_prefixEMT = 'cont_EMTracking_1';
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

%pointsToTake = 100;
pointsToTake = 0;


amountErrorPointsOT = 0;
amountErrorPointsEM = 0;

timeStampDivision = 10000000; %%change for more precise results,the smaller the number the more precise

for j = 1:numFiles %equals amount of OT-files (should be one with continuous measurement)
    % LOAD OT
    fileIDOT = fopen([file_path namesOT{j}],'r');
    dataArrayOT = textscan(fileIDOT, formatSpecOT, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition = [str2double(dataArrayOT{1,2}), str2double(dataArrayOT{1,3}), str2double(dataArrayOT{1,4})];
    TempOrient = [str2double(dataArrayOT{1,6}), str2double(dataArrayOT{1,7}), str2double(dataArrayOT{1,8}), str2double(dataArrayOT{1,9})];
    TimeStamp = str2double(dataArrayOT{1,11});
    TimeStamp = round(TimeStamp / timeStampDivision);
    % Stores OT in the cell
    numPointsOT = size(TempPosition,1)
    if pointsToTake~=0&&numPointsOT>pointsToTake
        numPointsOT=pointsToTake;
    end
    for k = 1:numPointsOT
         if (TempPosition(k,1) < 10000 && TempPosition(k,1) > -10000  && TempPosition(k,2) < 10000 && TempPosition(k,2) > -10000  && TempPosition(k,3) < 10000 && TempPosition(k,3) > -10000)
             dataOT{k-amountErrorPointsOT}.Position=TempPosition(k,:);
             dataOT{k-amountErrorPointsOT}.Orientation=TempOrient(k,:);
             dataOT{k-amountErrorPointsOT}.TimeStamp=TimeStamp(k);
         else
             amountErrorPointsOT = amountErrorPointsOT + 1;
         end
    end
    
   % clear TempPosition dataArrayOT TempOrient fileIDOT;
   
    % LOAD EM
    fileIDEM = fopen([file_path namesEM{j}],'r');
    dataArrayEM = textscan(fileIDEM, formatSpecEM, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
    TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
    % Split the raw data according to the 3 sensors
    TimeStamp = str2double(dataArrayEM{1,13});
    TimeStamp = round((TimeStamp+timeshift) / timeStampDivision);
    index0 = 1; index1 = 1; index2 = 1;
    for i = 1:size(TempPosition)
        switch str2double(dataArrayEM{1,2}(i));
            case 0
                TempPositionFirstSensor(index0,:) = TempPosition(i,:);
                TempOrientFirstSensor(index0,:) = TempOrient(i,:);
                TempTimeFirstSensor(index0) = TimeStamp(i);
                index0 = index0 + 1;
        end
    end
       
    % Stores EM in the cell
    numPointsEM = size(TempPositionFirstSensor,1)
    if pointsToTake~=0&&numPointsEM>pointsToTake
        numPointsEM=pointsToTake;
    end
    for k = 1:numPointsEM
        if (TempPositionFirstSensor(k,1) < 10000 && TempPositionFirstSensor(k,1) > -10000  && TempPositionFirstSensor(k,2) < 10000 && TempPositionFirstSensor(k,2) > -10000  && TempPositionFirstSensor(k,3) < 10000 && TempPositionFirstSensor(k,3) > -10000)
            dataEM{k-amountErrorPointsEM}.FirstSensor.Position=TempPositionFirstSensor(k,:);
            dataEM{k-amountErrorPointsEM}.FirstSensor.Orientation=TempOrientFirstSensor(k,:);
            dataEM{k-amountErrorPointsEM}.FirstSensor.TimeStamp=TempTimeFirstSensor(k);
        else
            amountErrorPointsEM = amountErrorPointsEM + 1;
        end
    end

%we just have 1 sensor    

   % clear TempPositionFirstSensor TempOrientFirstSensor TempPositionSecondSensor TempOrientSecondSensor TempPositionThirdSensor TempOrientThirdSensor TempPosition TempOrient dataArrayEM FirstSensor minimum fileIDEM;
end % Loader

%clear dEM dOT delimiter file_path file_prefixEMT file_prefixOT formatSpecOT formatSpecEM j namesEM namesOT dataEMTemp2P dataEMTemp2O index0 index1 index2;


%% Cells to Point-arrays
% Saves the computed points in matrices
%usually
numPointsEM = numPointsEM - amountErrorPointsEM;
numPointsOT = numPointsOT - amountErrorPointsOT;

% if(numPointsOT < numPointsEM)
%     numPoints = numPointsOT;
% else
%     numPoints = numPointsEM;
% end

figure
%plot measurements on time axis - should be alternating then
for k = 1:numPointsOT
  plot(dataOT{k}.TimeStamp, 'bx');
  hold on
end
for k = 1:numPointsEM
  plot(dataEM{k}.FirstSensor.TimeStamp, 'rx');
  hold on
end

%discretized

% goodPoints = 1:numPoints;
% %discard the other measurements
% % goodPoints = [1 2 6 10];
% numPoints = length(goodPoints);
% 
% opticalPoints = zeros(3, numPoints);
% emPointsFirstSensor = zeros(3,numPoints);

%opticalOrientations = zeros(4, numPoints);
%emOrientationsFirstSensor = zeros(4,numPoints);

% determine 3D positions of both sensors (dataEM and dataOTToEMOne) for
% each time step, we only want the times where we have data of both
% tracking devices
minTSOT = dataOT{1}.TimeStamp;
maxTSOT = dataOT{numPointsOT}.TimeStamp;
minTSEM = dataEM{1}.FirstSensor.TimeStamp;
maxTSEM = dataEM{numPointsEM}.FirstSensor.TimeStamp;

iOT = 1;
counterOT = 1;
for t = minTSOT:maxTSOT
    if dataOT{iOT}.TimeStamp == t
        allOT{counterOT}.Position = dataOT{iOT}.Position;
        allOT{counterOT}.Orientation = dataOT{iOT}.Orientation;
        allOT{counterOT}.TimeStamp = dataOT{iOT}.TimeStamp;
        iOT = iOT + 1;
    else
        %position needs to be interpolated between iOT and iOT - 1
        allOT{counterOT}.TimeStamp = t;
        alpha =(dataOT{iOT}.TimeStamp - t) / (dataOT{iOT}.TimeStamp - dataOT{iOT-1}.TimeStamp);
        allOT{counterOT}.Position(1) = (1-alpha) * dataOT{iOT}.Position(1) + alpha * dataOT{iOT-1}.Position(1);
        allOT{counterOT}.Position(2) = (1-alpha) * dataOT{iOT}.Position(2) + alpha * dataOT{iOT-1}.Position(2);
        allOT{counterOT}.Position(3) = (1-alpha) * dataOT{iOT}.Position(3) + alpha * dataOT{iOT-1}.Position(3);
        for o =1:4
            allOT{counterOT}.Orientation(o) = (1-alpha) * dataOT{iOT}.Orientation(o) + alpha * dataOT{iOT-1}.Orientation(o);
        end
    end
    counterOT = counterOT +1;
end

iEM = 1;
counterEM = 1;
for t = minTSEM:maxTSEM
    if dataEM{iEM}.FirstSensor.TimeStamp == t
        allEM{counterEM}.Position = dataEM{iEM}.FirstSensor.Position;
        allEM{counterEM}.TimeStamp = dataEM{iEM}.FirstSensor.TimeStamp;
        iEM = iEM + 1;
    else
        %position needs to be interpolated between iOT and iOT - 1
        allEM{counterEM}.TimeStamp = t;
        alpha =(dataEM{iEM}.FirstSensor.TimeStamp - t) / (dataEM{iEM}.FirstSensor.TimeStamp - dataEM{iEM-1}.FirstSensor.TimeStamp);
        allEM{counterEM}.Position(1) = (1-alpha) * dataEM{iEM}.FirstSensor.Position(1) + alpha * dataEM{iEM-1}.FirstSensor.Position(1);
        allEM{counterEM}.Position(2) = (1-alpha) * dataEM{iEM}.FirstSensor.Position(2) + alpha * dataEM{iEM-1}.FirstSensor.Position(2);
        allEM{counterEM}.Position(3) = (1-alpha) * dataEM{iEM}.FirstSensor.Position(3) + alpha * dataEM{iEM-1}.FirstSensor.Position(3);
        for o=1:4
            allEM{counterEM}.Orientation(o) = (1-alpha) * dataEM{iEM}.FirstSensor.Orientation(o) + alpha * dataEM{iEM-1}.FirstSensor.Orientation(o);
        end
    end
    counterEM = counterEM +1;
end


if minTSOT > minTSEM
    min = minTSOT;
else
    min = minTSEM;
end
if maxTSOT < maxTSEM
    max = maxTSOT;
else
    max = maxTSEM;
end

% determine boundaries
minEM=1;
if min~=minTSEM
    for i = 1:counterEM-1
        if allEM{i}.TimeStamp == min
            minEM=i;
        end
    end
end
minOT=1;
if min~=minTSOT
    for i = 1:counterOT-1
        if allOT{i}.TimeStamp == min
            minOT=i;
        end
    end
end
maxEM=counterEM-1;
if max~=maxTSEM
    for i = 1:counterEM-1
        if allEM{i}.TimeStamp == max
            maxEM=i;
        end
    end
end
maxOT=counterOT-1;
if max~=maxTSOT
    for i = 1:counterOT-1
        if allOT{i}.TimeStamp == max
            maxOT=i;
        end
    end
end   


counter=1;
for i = minOT:maxOT
    OTtoUse{counter} = allOT{i};
    counter= counter+1;
end
counter = 1;
for i = minEM:maxEM
    EMtoUse{counter} = allEM{i};
    counter= counter+1;
end

figure %both in their coordinates..
for i = 1:counter-1
    plot3(EMtoUse{i}.Position(1), EMtoUse{i}.Position(2), EMtoUse{i}.Position(3), 'ro');
    hold on;
    box on;
    grid on;
end
for i = 1:counter-1
    plot3(OTtoUse{i}.Position(1), OTtoUse{i}.Position(2), OTtoUse{i}.Position(3), 'bx');
    hold on;
end

