%% LOADER and parser
close all, clear all;
clc;

file_path = '..\timeshift\05.29 Measurements\';
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

%take all points in the file
pointsToTake = 0;

amountErrorPointsOT = 0;
amountErrorPointsEM = 0;

% define desired polling frequency
datafreq = 100; % 100 Hz

% calc sample period
dataperiod = 1/datafreq;
%in nanoseconds
dataperiod = dataperiod * 1e9;

% now the samples can have a maximum temporal resolution of 'datafreq'
% hertz

timeStampDivision = dataperiod; %%change for more precise results,the smaller the number the more precise

for j = 1:numFiles %equals amount of OT-files (each file represents several measurements of one point, in 04.23 set 2 we have 6 points)
    % LOAD OT
    fileIDOT = fopen([file_path namesOT{j}],'r');
    dataArrayOT = textscan(fileIDOT, formatSpecOT, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition = [str2double(dataArrayOT{1,2}), str2double(dataArrayOT{1,3}), str2double(dataArrayOT{1,4})];
    TempOrient = [str2double(dataArrayOT{1,6}), str2double(dataArrayOT{1,7}), str2double(dataArrayOT{1,8}), str2double(dataArrayOT{1,9})];
    TimeStamp = str2double(dataArrayOT{1,11});
    if all(TimeStamp > timeStampDivision)
        if j==1
            TimeStampOffsetOT = round(TimeStamp(1) / timeStampDivision);
        end
%             TimeStamp = round(TimeStamp / timeStampDivision) - TimeStampOffsetOT;
            TimeStamp = round(TimeStamp / timeStampDivision);
    else
        error('Value of TimeStamp is lower than sampling period')
    end
    % Stores OT in the cell
    numPointsOT = size(TempPosition,1)
    if pointsToTake~=0&&numPointsOT>pointsToTake
        numPointsOT=pointsToTake;
    end
    for k = 1:numPointsOT
         if (TempPosition(k,1) < 10000 && TempPosition(k,1) > -10000  && TempPosition(k,2) < 10000 && TempPosition(k,2) > -10000  && TempPosition(k,3) < 10000 && TempPosition(k,3) > -10000)
             dataOT{k-amountErrorPointsOT,1}.position=TempPosition(k,:);
             dataOT{k-amountErrorPointsOT,1}.orientation=TempOrient(k,:);
             dataOT{k-amountErrorPointsOT,1}.TimeStamp=TimeStamp(k);
         else
             amountErrorPointsOT = amountErrorPointsOT + 1;
         end
    end
   
    % LOAD EM
    fileIDEM = fopen([file_path namesEM{j}],'r');
    dataArrayEM = textscan(fileIDEM, formatSpecEM, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
    TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
    
    % Split the raw data according to the 3 sensors
    TimeStamp = str2double(dataArrayEM{1,13});
    if all(TimeStamp > timeStampDivision)
        if j==1
            TimeStampOffsetEM = round(TimeStamp(1) / timeStampDivision);
        end
%             TimeStamp = round(TimeStamp / timeStampDivision) - TimeStampOffsetEM;
            TimeStamp = round(TimeStamp / timeStampDivision);
    else
        error('Value of TimeStamp is lower than sampling period')
    end
    index0 = 1; index1 = 1; index2 = 1;
    for i = 1:size(TempPosition)
        switch str2double(dataArrayEM{1,2}(i));
            % only take sensor 1
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
            dataEM{k-amountErrorPointsEM,1}.position=TempPositionFirstSensor(k,:);
            dataEM{k-amountErrorPointsEM,1}.orientation=TempOrientFirstSensor(k,:);
            dataEM{k-amountErrorPointsEM,1}.TimeStamp=TempTimeFirstSensor(k);
        else
            amountErrorPointsEM = amountErrorPointsEM + 1;
        end
    end
end % Loader
%% plot loaded timestamps
numPointsOT = numPointsOT-amountErrorPointsOT
numPointsEM = numPointsEM-amountErrorPointsEM

comparetimestamps_fighandle = figure;
hold on
for k = 1:numPointsOT
    plot(k, dataOT{k}.TimeStamp/datafreq, 'rx')
end
hold off

hold on
for k = 1:numPointsEM
    plot(k+100, dataOT{k}.TimeStamp/datafreq, 'bo')
end
hold off



    
    