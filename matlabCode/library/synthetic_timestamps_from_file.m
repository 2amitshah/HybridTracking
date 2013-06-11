%syntethic_timestamps_from_file interpolates the position vector to obtain the
%location in a different timestamp
%
%INPUT
%
%path:
%Data with the position of the device. Txt file (recorded from the C++
%application TrackingFusion CAMPCom).
%
%interval:
%(1x2) vector with the initial and final point of the new timestamp
%
% frequency:
% frequency of the new measurements. Step can be obtained as 1/frequency
% 
%OUTPUT
%
%dataOutput:
%Cell with dimensions {numSensors}. Each field contains a position and
%timestamp array. Timestamps gives the values of the new transformation,
%and position has a dimension of (numPoints,3), with numPoints 
%is the given by the new timestamp vector, and 3 corresponds to the
%position in x, y, z respectively.
%
%Author: Santiago Perez, June 2013

function dataOutput = synthetic_timestamps_from_file( path, interval, frequency )

numSensors = 1; % Default, for OT
dataOutput = cell(1,numSensors);
offset = 8;

% Load from txt file
data_raw = importdata(path,' ');
if (size(data_raw.textdata,2) == 10)
    typeData = 'OT';
elseif (size(data_raw.textdata,2) == 12)
    typeData = 'EM';
else
    disp('error?????');
end

% Parsing. We have to check why everything but time is obtained in string
timestamps_original_vector{numSensors} = data_raw.data;
position_original_string = (data_raw.textdata(:,end-offset:end-offset+2)); % Data comes in cells

% Conversion cell to double. Propietary library (in toolboxes, CStr2String).
% Faster than with str2double!!
% Storage in a cell for different sensors purposed and homogeneity
posX = sscanf(CStr2String(position_original_string(:,1),'*'),'%f*');
posY = sscanf(CStr2String(position_original_string(:,2),'*'),'%f*');
posZ = sscanf(CStr2String(position_original_string(:,3),'*'),'%f*');
position_original_vector{numSensors} = [posX posY posZ];

% If we are analysing EM, we need to separate each sensor.
if (strcmp(typeData,'EM'))
    sensor_original_vector = sscanf(CStr2String(data_raw.textdata(:,2),'*'),'%f*');
    numSensors = max(sensor_original_vector) + 1;
    for i = 1:numSensors
        position_original_temp{i} = position_original_vector{1}(sensor_original_vector==i-1,:);
        timestamps_original_temp{i} = timestamps_original_vector{1}(sensor_original_vector==i-1,:);
    end
    position_original_vector = position_original_temp;
    timestamps_original_vector = timestamps_original_temp;
    
    clear position_original_temp timestamps_original_temp sensor_original_vector;
end

% Free memory
clear data_raw posX posY posZ position_original_string;




begin_TS_ns = interval(1);
end_TS_ns = interval(2);
step = 1e9/frequency;

timestampsNewVector = [begin_TS_ns:step:end_TS_ns];

for i = 1:numSensors
    dataOutput{i}.position = [interp1(timestamps_original_vector{i},position_original_vector{i}(:,1),timestampsNewVector) %x
        interp1(timestamps_original_vector{i},position_original_vector{i}(:,2),timestampsNewVector)     %y
        interp1(timestamps_original_vector{i},position_original_vector{i}(:,3),timestampsNewVector)]';  %z
    dataOutput{i}.timestamp = timestampsNewVector';
end

end

