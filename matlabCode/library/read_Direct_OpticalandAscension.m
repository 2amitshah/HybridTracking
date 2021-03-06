function [dataOT, dataEM_ASC, errorTimeStampsOT, errorTimeStampsEM_ASC] = read_Direct_OpticalAndAscension(file_path, file_prefixOT, file_prefixEMT_ASC, verbosity)

% definitions
if ~exist('verbosity', 'var')
    verbosity = 'vRelease';
end

filenames_struct = file_path;
if isstruct(filenames_struct)
    file_prefixEMT_ASC = filenames_struct.EMfiles;
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
if ~exist('file_prefixEMT_ASC', 'var')
    file_prefixEMT_ASC = 'EMTrackingAscensionDirect_3';
end
dOT = dir([file_path filesep file_prefixOT '*']);
dEM = dir([file_path filesep file_prefixEMT_ASC '*']);

numFiles = numel(dOT);
namesOT = sort({dOT(:).name});
namesEM = sort({dEM(:).name});

errorTimeStampsOT = cell(1);
errorTimeStampsEM_ASC = cell(1);
amountErrorPointsOT = 0;
amountErrorPointsEM = [0 0 0];
goodOTPts = cell(1);
badOTPts = cell(1);
goodEMPts = cell(1,4);
badEMPts = cell(1,4);
indexCounterEM_ASC = [0 0 0 0];

delimiter = ' ';
formatSpecOT = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
formatSpecEM_ASC = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

dataOT = cell(numFiles,1);
dataEM_ASC = cell(numFiles,1);


    
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
    dataArrayEM = textscan(fileIDEM, formatSpecEM_ASC, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    fclose(fileIDEM);
    TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
    TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
    TimeStampEM = str2double(dataArrayEM{1,17});
    DeviceTimeStampEM = str2double(dataArrayEM{1,15});
    % Store EM in a cell struct
    for i = 1:size(TempPosition,1)
        SensorIndex0based = int8(str2double(dataArrayEM{1,2}(i)));
        SensorIndex=SensorIndex0based+1;
        indexCounterEM_ASC(SensorIndex) = indexCounterEM_ASC(SensorIndex) + 1;
        % Split the raw data according to the 3 sensors
        if ( abs(TempPosition(i,1)) < 10000 )
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.position=TempPosition(i,:);
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.orientation=TempOrient(i,:);
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.TimeStamp=TimeStampEM(i);
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.DeviceTimeStamp=DeviceTimeStampEM(i);
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.valid = 1;
            
            goodEMPts{1,SensorIndex} = [goodEMPts{1,SensorIndex} indexCounterEM_ASC(SensorIndex)];
        else
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.position=[];
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.orientation=[];
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.TimeStamp=TimeStampEM(i);
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.DeviceTimeStamp=DeviceTimeStampEM(i);
            dataEM_ASC{indexCounterEM_ASC(SensorIndex),SensorIndex}.valid = 0;
            
            badEMPts{1,SensorIndex} = [badEMPts{1,SensorIndex} indexCounterEM_ASC(SensorIndex)];
            amountErrorPointsEM(SensorIndex) = amountErrorPointsEM(SensorIndex) + 1;
            errorTimeStampsEM_ASC{amountErrorPointsEM(SensorIndex), SensorIndex} = TimeStampEM(i);

        end
    end
    
    
    
    numPointsEMT = size(dataEM_ASC,1);
    numSensorsEMT = size(dataEM_ASC,2);
    disp(['Raw EM data - Points: ' num2str(numPointsEMT) ' Sensors: ' num2str(numSensorsEMT)])
    
    
    if strcmp(verbosity, 'vDebug')
        % for EM
        % network Timestamp
        timestampsEM = zeros(size(dataEM_ASC));
        for j = 1:numSensorsEMT
            for i = 1:numPointsEMT
                if ~isempty(dataEM_ASC{i,j})
                    timestampsEM(i,j) = dataEM_ASC{i,j}.TimeStamp;
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
        DeviceTimestampsEM = zeros(size(dataEM_ASC));
        for j = 1:numSensorsEMT
            for i = 1:numPointsEMT
                if ~isempty(dataEM_ASC{i,j})
                    DeviceTimestampsEM(i,j) = dataEM_ASC{i,j}.DeviceTimeStamp;
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