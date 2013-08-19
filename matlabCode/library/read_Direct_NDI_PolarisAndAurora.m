function [dataOT, dataEM, errorTimeStampsOT, errorTimeStampsEM] = read_Direct_NDI_PolarisAndAurora(file_path, verbosity)
% Reads in new format compared to read_TrackingFusion_files.m . Here,
% aditionally to position, Orientation and Network Timestamp, the Device
% Timestamps and the NDI Error value are read in.

% definitions
if ~exist('verbosity', 'var')
    verbosity = 'vRelease';
end

filenames_struct = file_path;
if isstruct(filenames_struct)
    file_prefixEMT = filenames_struct.EMfiles;
    file_prefixOT = filenames_struct.OTfiles;
    file_path = filenames_struct.folder;
end

if ~exist('file_path', 'var')
    % read_TrackingFusion_files should be located in
    % HybridTracking\matlabCode\library\
    pathGeneral = fileparts(fileparts(fileparts(which(mfilename)))); % pathGeneral is HybridTracking\
    file_path = [pathGeneral filesep 'measurements' filesep '07.29_Measurements'];
end
if ~exist('file_prefixOT', 'var')
    file_prefixOT = 'OpticalTrackingDirect';
end
if ~exist('file_prefixEMT', 'var')
    file_prefixEMT = 'EMTrackingAscensionDirect_3';
end
dOT = dir([file_path filesep file_prefixOT '*']);
dEM = dir([file_path filesep file_prefixEMT '*']);

numFiles = numel(dOT);
namesOT = sort({dOT(:).name});
namesEM = sort({dEM(:).name});

errorTimeStampsOT = cell(1);
errorTimeStampsEM = cell(1);
amountErrorPointsOT = 0;
amountErrorPointsEM = [0 0 0];
goodOTPts = cell(1);
badOTPts = cell(1);
goodEMPts = cell(1,4);
badEMPts = cell(1,4);
indexCounterEM = [0 0 0 0];

delimiter = ' ';
formatSpecOT = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
formatSpecEM = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

dataOT = cell(numFiles,1);
dataEM = cell(numFiles,1);
   
%(there is only one Tracking file)
    
% LOAD OT
fileIDOT = fopen([file_path filesep namesOT{1}],'r');
dataArrayOT = textscan(fileIDOT, formatSpecOT, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
fclose(fileIDOT);
TempPosition = [str2double(dataArrayOT{1,2}), str2double(dataArrayOT{1,3}), str2double(dataArrayOT{1,4})];
TempOrient = [str2double(dataArrayOT{1,6}), str2double(dataArrayOT{1,7}), str2double(dataArrayOT{1,8}), str2double(dataArrayOT{1,9})];
TimeStampOT = str2double(dataArrayOT{1,15});
DeviceTimeStampOT = str2double(dataArrayOT{1,13}) / 60;
% Store OT in a cell struct
numPointsOT = size(TempPosition,1);
for k = 1:numPointsOT
     if ( abs(TempPosition(k,1)) < 10000 )
         dataOT{k,1}.position=TempPosition(k,:);
         dataOT{k,1}.orientation=TempOrient(k,:);
         dataOT{k,1}.TimeStamp=TimeStampOT(k);
         dataOT{k,1}.DeviceTimeStamp=DeviceTimeStampOT(k);
         dataOT{k,1}.valid=1;

         goodOTPts{1} = [goodOTPts{1} k];
     else
         dataOT{k,1}.position=[];
         dataOT{k,1}.orientation=[];
         dataOT{k,1}.TimeStamp=TimeStampOT(k);
         dataOT{k,1}.DeviceTimeStamp=DeviceTimeStampOT(k);
         dataOT{k,1}.valid=0;

         badOTPts{1} = [badOTPts{1} k];
         amountErrorPointsOT = amountErrorPointsOT + 1;
         errorTimeStampsOT{amountErrorPointsOT} = TimeStampOT(k);
     end
end
numPointsOT = size(dataOT,1);
disp(['Raw Optical data - Points: ' num2str(numPointsOT)])

% LOAD EM
fileIDEM = fopen([file_path filesep namesEM{1}],'r');
dataArrayEM = textscan(fileIDEM, formatSpecEM, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
fclose(fileIDEM);
TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
TimeStampEM = str2double(dataArrayEM{1,17});
% Aurora System has a tricky framerate scheme (from Aurora API guide, p.13/101, 2.6):

% Use the frame number, and not the host computer clock, to identify when data was collected.
% The frame number is incremented by 8 at a constant rate of 40 Hz. Associating a time from
% the host computer clock to replies from the system assumes that the duration of time
% between raw data collection and when the reply is received by the host computer is constant.
% This is not necessarily the case. The frame number is returned with the command BX
% (page 12) or TX (page 63).

% So to get to seconds, we divide by 8 and then by 40
DeviceTimeStampEM = str2double(dataArrayEM{1,15}) / (8*40);
% Store EM in a cell struct
for i = 1:size(TempPosition,1)
    SensorIndex0based = int8(str2double(dataArrayEM{1,2}(i)));
    SensorIndex=SensorIndex0based+1;
    indexCounterEM(SensorIndex) = indexCounterEM(SensorIndex) + 1;
    % Split the raw data according to the 3 sensors
    if ( abs(TempPosition(i,1)) < 10000 )
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.position=TempPosition(i,:);
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.orientation=TempOrient(i,:);
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.TimeStamp=TimeStampEM(i);
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.DeviceTimeStamp=DeviceTimeStampEM(i);
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.valid = 1;

        goodEMPts{1,SensorIndex} = [goodEMPts{1,SensorIndex} indexCounterEM(SensorIndex)];
    else
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.position=[];
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.orientation=[];
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.TimeStamp=TimeStampEM(i);
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.DeviceTimeStamp=DeviceTimeStampEM(i);
        dataEM{indexCounterEM(SensorIndex),SensorIndex}.valid = 0;

        badEMPts{1,SensorIndex} = [badEMPts{1,SensorIndex} indexCounterEM(SensorIndex)];
        amountErrorPointsEM(SensorIndex) = amountErrorPointsEM(SensorIndex) + 1;
        errorTimeStampsEM{amountErrorPointsEM(SensorIndex), SensorIndex} = TimeStampEM(i);

    end
end

numPointsEMT = size(dataEM,1);
numSensorsEMT = size(dataEM,2);
disp(['Raw EM data - Points: ' num2str(numPointsEMT) ' Sensors: ' num2str(numSensorsEMT)])

% Plot Timestamp statistics
if strcmp(verbosity, 'vDebug')
    % for EM
    % network Timestamp
    timestampsEM = zeros(size(dataEM));
    for j = 1:numSensorsEMT
        for i = 1:numPointsEMT
            if ~isempty(dataEM{i,j})
                timestampsEM(i,j) = dataEM{i,j}.TimeStamp;
            end
        end
    end

    timestampsEM = timestampsEM/1e9;

    TimeDiff_EM_fig = figure;
    for j = 1:numSensorsEMT
    subplot(2,numSensorsEMT,j)
    lastValidIndex = find(timestampsEM(:,j) ~= 0, 1, 'last');
    plot((timestampsEM(2:lastValidIndex,j)-timestampsEM(1:(lastValidIndex-1),j)))
    title(['Network Time difference between consecutive readings from sensor ' num2str(j) ' in seconds. Average should be at 0,02 s ~ 50 Hz'])
    end

    sorted_timestampsEM = sort(timestampsEM(:));
    firstValidIndex = find(sorted_timestampsEM ~= 0, 1, 'first');
    subplot(2,numSensorsEMT, (numSensorsEMT+1):(2*numSensorsEMT))
    plot((sorted_timestampsEM((firstValidIndex+1):end)-sorted_timestampsEM(firstValidIndex:(end-1))))
    title('Network Time difference between all consecutive EMT readings in seconds. Maximum should be at 0,02 s ~ 50 Hz')

    % Device Timestamp
    DeviceTimestampsEM = zeros(size(dataEM));
    for j = 1:numSensorsEMT
        for i = 1:numPointsEMT
            if ~isempty(dataEM{i,j})
                DeviceTimestampsEM(i,j) = dataEM{i,j}.DeviceTimeStamp;
            end
        end
    end

    DeviceTimeDiff_EM_fig = figure;
    for j = 1:numSensorsEMT
    subplot(2,numSensorsEMT,j)
    lastValidIndex = find(DeviceTimestampsEM(:,j) ~= 0, 1, 'last');
    plot((DeviceTimestampsEM(2:lastValidIndex,j)-DeviceTimestampsEM(1:(lastValidIndex-1),j)))
    title(['Device Time difference between consecutive readings from sensor ' num2str(j) ' in seconds. Average should be at 0,02 s ~ 50 Hz'])
    end

    sorted_DeviceTimestampsEM = sort(DeviceTimestampsEM(:));
    firstValidIndex = find(sorted_DeviceTimestampsEM ~= 0, 1, 'first');
    subplot(2,numSensorsEMT, (numSensorsEMT+1):(2*numSensorsEMT))
    plot((sorted_DeviceTimestampsEM((firstValidIndex+1):end)-sorted_DeviceTimestampsEM(firstValidIndex:(end-1))))
    title('Device Time difference between all consecutive EMT readings in seconds. Maximum should be at 0,02 s ~ 50 Hz')

    % for OT        
    TimeStampOT_sec = TimeStampOT/1e9;

    TimeDiff_OT_fig = figure;
    plot((TimeStampOT_sec(2:end)-TimeStampOT_sec(1:(end-1))))
    title('Network Time difference between all consecutive OT readings in seconds. Average should be at 0,5 s ~ 20 Hz')

    FrameDiff_OT_fig = figure;
    plot((DeviceTimeStampOT(2:end)-DeviceTimeStampOT(1:(end-1))))
    title('Device Time difference between all consecutive OT readings in seconds. Average should be at 3 Frames ~ 60/3 = 20 Hz = 0,05 s')
    ylim([0 0.1])

end
    

end