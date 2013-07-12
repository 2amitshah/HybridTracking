function [dataOT, dataEM, errorTimeStampsOT, errorTimeStampsEM] = read_TrackingFusion_files(file_path, file_prefixOT, file_prefixEMT, timeStampDiv)

% definitions

if ~exist('file_path', 'var')
    % read_TrackingFusion_files should be located in
    % HybridTracking\matlabCode\library\
    pathGeneral = fileparts(fileparts(fileparts(which(mfilename)))); % pathGeneral is HybridTracking\
    file_path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];
end
if ~exist('file_prefixOT', 'var')
    file_prefixOT = 'OpticalTracking_cont';
end
if ~exist('file_prefixEMT', 'var')
    file_prefixEMT = 'EMTracking_cont';
end
dOT = dir([file_path filesep file_prefixOT '*']);
dEM = dir([file_path filesep file_prefixEMT '*']);

numFiles = numel(dOT);
namesOT = sort({dOT(:).name});
namesEM = sort({dEM(:).name});

errorTimeStampsOT = cell(1);
errorTimeStampsEM = cell(1);
amountErrorPointsOT = 0;
amountErrorPointsEM = [0 0];
goodOTPts = cell(1);
badOTPts = cell(1);
goodEMPts = cell(1,3);
badEMPts = cell(1,3);
indexCounterEM = [0 0 0];

delimiter = ' ';
formatSpecOT = '%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
formatSpecEM = '%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

dataOT = cell(numFiles,1);
dataEM = cell(numFiles,1);

if numFiles~=1
    for j = 1:numFiles %equals amount of OT-files (each file represents several measurements of one point, in 04.23 set 2 we have 6 points)
    indexCounterEM = [0 0 0];
    amountErrorPointsOT = 0;
    amountErrorPointsEM = [0 0];
    goodOTPts = cell(1);
    badOTPts = cell(1);
    goodEMPts = cell(1,3);
    badEMPts = cell(1,3);
    dataOT_all = cell(1,1);
    dataEM_all = cell(1,1);
        % LOAD OT
        fileIDOT = fopen([file_path filesep namesOT{j}],'r');
        dataArrayOT = textscan(fileIDOT, formatSpecOT, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
        fclose(fileIDOT);
        TempPosition = [str2double(dataArrayOT{1,2}), str2double(dataArrayOT{1,3}), str2double(dataArrayOT{1,4})];
        TempOrient = [str2double(dataArrayOT{1,6}), str2double(dataArrayOT{1,7}), str2double(dataArrayOT{1,8}), str2double(dataArrayOT{1,9})];
        TimeStampOT = str2double(dataArrayOT{1,11});
        % Store OT in a cell struct
        numPointsOT = size(TempPosition,1);
        for k = 1:numPointsOT
             if ( abs(TempPosition(k,1)) < 10000 )
                 dataOT_all{k,1}.position=TempPosition(k,:);
                 dataOT_all{k,1}.orientation=TempOrient(k,:);
                 dataOT_all{k,1}.TimeStamp=TimeStampOT(k);
                 dataOT_all{k,1}.valid=1;
                 goodOTPts{1} = [goodOTPts{1} k];
             else
                 dataOT_all{k,1}.position=[];
                 dataOT_all{k,1}.orientation=[];
                 dataOT_all{k,1}.valid=0;
                 dataOT_all{k,1}.TimeStamp=TimeStampOT(k);
                 amountErrorPointsOT = amountErrorPointsOT + 1;
                 errorTimeStampsOT{amountErrorPointsOT} = TimeStampOT(k);
                 badOTPts{1} = [badOTPts{1} k];
             end
        end
        
        maxnum_OTPts = size(dataOT_all,1);
        
        positions = zeros(maxnum_OTPts,3);
        orientations = zeros(maxnum_OTPts,4);
        
        for i = 1:maxnum_OTPts
            if dataOT_all{i,1}.valid
                positions(i,:) = dataOT_all{i}.position;
                orientations(i,:) = dataOT_all{i}.orientation;
            end
        end
        
        dataOT{j}.position = mean(positions(goodOTPts{1},:),1);
        dataOT{j}.orientation = mean(orientations(goodOTPts{1},:),1);

        % LOAD EM
        fileIDEM = fopen([file_path filesep namesEM{j}],'r');
        dataArrayEM = textscan(fileIDEM, formatSpecEM, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
        fclose(fileIDEM);
        TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
        TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
        TimeStampEM = str2double(dataArrayEM{1,13});
        % Store EM in a cell struct
        for i = 1:size(TempPosition,1)
            SensorIndex0based = int8(str2double(dataArrayEM{1,2}(i)));
            SensorIndex=SensorIndex0based+1;
            indexCounterEM(SensorIndex) = indexCounterEM(SensorIndex) + 1;
            % Split the raw data according to the 3 sensors
            if ( abs(TempPosition(i,1)) < 10000 )
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.position=TempPosition(i,:);
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.orientation=TempOrient(i,:);
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.TimeStamp=TimeStampEM(i);
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.valid = 1;
                goodEMPts{1,SensorIndex} = [goodEMPts{1,SensorIndex} indexCounterEM(SensorIndex)];
            else
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.position=[];
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.orientation=[];
                amountErrorPointsEM(SensorIndex) = amountErrorPointsEM(SensorIndex) + 1;
                errorTimeStampsEM{amountErrorPointsEM(SensorIndex), SensorIndex} = TimeStampEM(i);
                badEMPts{1,SensorIndex} = [badEMPts{1,SensorIndex} indexCounterEM(SensorIndex)];
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.valid = 0;
                dataEM_all{indexCounterEM(SensorIndex),SensorIndex}.TimeStamp=TimeStampEM(i);
            end
        end
        num_EMSensors = size(dataEM_all,2);
        maxnum_EMPts = size(dataEM_all,1);
        
        position = cell(1,num_EMSensors);
        orientation = cell(1,num_EMSensors);
        timestamps = cell(1,num_EMSensors);
        for i = 1:num_EMSensors
            position{i}= zeros(maxnum_EMPts,3);
            orientation{i}= zeros(maxnum_EMPts,4);
            timestamps{i} = zeros(maxnum_EMPts,1);
        end
        
        for i = 1:maxnum_EMPts %nr of measurement
            for k = 1:num_EMSensors % sensor
                if dataEM_all{i,k}.valid
                position{k}(i,:) = dataEM_all{i,k}.position;
                orientation{k}(i,:) = dataEM_all{i,k}.orientation;
                timestamps{k}(i) = dataEM_all{i,k}.TimeStamp;
                end
            end
        end

        for k = 1:num_EMSensors
            dataEM{j,k}.position = mean(position{k}(goodEMPts{k},:),1);
            dataEM{j,k}.orientation = mean(orientation{k}(goodEMPts{k},:),1);
            dataEM{j,k}.valid = 1;
            dataEM{j,k}.TimeStamp = round(mean(timestamps{k}(goodEMPts{k},:),1));
%             dataEM{j,k}.position = mean(position{k}, 1);
%             dataEM{j,k}.orientation = mean(orientation{k}, 1);
        end

    end %for loop of Tracking files
    
else %(there is only one Tracking file)
    
    % LOAD OT
    fileIDOT = fopen([file_path filesep namesOT{1}],'r');
    dataArrayOT = textscan(fileIDOT, formatSpecOT, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    fclose(fileIDOT);
    TempPosition = [str2double(dataArrayOT{1,2}), str2double(dataArrayOT{1,3}), str2double(dataArrayOT{1,4})];
    TempOrient = [str2double(dataArrayOT{1,6}), str2double(dataArrayOT{1,7}), str2double(dataArrayOT{1,8}), str2double(dataArrayOT{1,9})];
    TimeStampOT = str2double(dataArrayOT{1,11});
    % Store OT in a cell struct
    numPointsOT = size(TempPosition,1);
    for k = 1:numPointsOT
         if ( abs(TempPosition(k,1)) < 10000 )
             dataOT{k,1}.position=TempPosition(k,:);
             dataOT{k,1}.orientation=TempOrient(k,:);
             dataOT{k,1}.TimeStamp=TimeStampOT(k);
             dataOT{k,1}.valid=1;
             goodOTPts{1} = [goodOTPts{1} k];
         else
             dataOT{k,1}.position=[];
             dataOT{k,1}.orientation=[];
             amountErrorPointsOT = amountErrorPointsOT + 1;
             errorTimeStampsOT{amountErrorPointsOT} = TimeStampOT(k);
             dataOT{k,1}.valid=0;
             dataOT{k,1}.TimeStamp=TimeStampOT(k);
             badOTPts{1} = [badOTPts{1} k];
         end
    end
    numPointsOT = size(dataOT,1)

    % LOAD EM
    fileIDEM = fopen([file_path filesep namesEM{1}],'r');
    dataArrayEM = textscan(fileIDEM, formatSpecEM, 'Delimiter', delimiter, 'MultipleDelimsAsOne', true,  'ReturnOnError', false);
    fclose(fileIDEM);
    TempPosition= [str2double(dataArrayEM{1,4}), str2double(dataArrayEM{1,5}), str2double(dataArrayEM{1,6})];
    TempOrient = [str2double(dataArrayEM{1,8}), str2double(dataArrayEM{1,9}), str2double(dataArrayEM{1,10}), str2double(dataArrayEM{1,11})];
    TimeStampEM = str2double(dataArrayEM{1,13});
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
            dataEM{indexCounterEM(SensorIndex),SensorIndex}.valid = 1;
            goodEMPts{1,SensorIndex} = [goodEMPts{1,SensorIndex} indexCounterEM(SensorIndex)];
        else
            dataEM{indexCounterEM(SensorIndex),SensorIndex}.position=[];
            dataEM{indexCounterEM(SensorIndex),SensorIndex}.orientation=[];
            amountErrorPointsEM(SensorIndex) = amountErrorPointsEM(SensorIndex) + 1;
            errorTimeStampsEM{amountErrorPointsEM(SensorIndex), SensorIndex} = TimeStampEM(i);
            badEMPts{1,SensorIndex} = [badEMPts{1,SensorIndex} indexCounterEM(SensorIndex)];
            dataEM{indexCounterEM(SensorIndex),SensorIndex}.valid = 0;
            dataEM{indexCounterEM(SensorIndex),SensorIndex}.TimeStamp=TimeStampEM(i);
        end
    end

    numPointsEMT = size(dataEM,1)
    numSensorsEM = size(dataEM,2)
    
end

end