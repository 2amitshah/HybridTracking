%syntethic_timestamps interpolates the position vector to obtain the
%location in a different timestamp
%
%INPUT
%
%path:
%Data with the position of the device.
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

function dataOutput = synthetic_timestamps( dataInput, interval, frequency )

numSensors = size(dataInput,2); % Default, for OT


timestamps_original_vector = cell(1,numSensors);
position_original_vector = cell(size(timestamps_original_vector));
for j = 1:numSensors
    timestamps_original_vector{j}(1,1)=dataInput{1,j}.TimeStamp;
    position_original_vector{j}(1,:)=dataInput{1,j}.position;
end


for j = 1:numSensors
    for i = 2:size(dataInput,1)
        if (~isempty(dataInput{i,j}) && dataInput{i,j}.TimeStamp > timestamps_original_vector{j}(end,1)) % Remove any duplicate position
            timestamps_original_vector{j}(end+1,1)=dataInput{i,j}.TimeStamp;
            position_original_vector{j}(end+1,:)=dataInput{i,j}.position;
        end
    end
end



begin_TS_ns = interval(1);
end_TS_ns = interval(2);
step = 1e9/frequency;
timestampsNewVector = [begin_TS_ns:step:end_TS_ns];




temporalPosition = cell(1,numSensors);
for i = 1:numSensors
    temporalPosition{i}.position = [interp1(timestamps_original_vector{i},position_original_vector{i}(:,1),timestampsNewVector) %x
        interp1(timestamps_original_vector{i},position_original_vector{i}(:,2),timestampsNewVector)     %y
        interp1(timestamps_original_vector{i},position_original_vector{i}(:,3),timestampsNewVector)]';  %z
end


% arrange on the same format
dataOutput = cell(numel(timestampsNewVector),numSensors);
for j = 1:numel(timestampsNewVector)
    for i = 1:numSensors
        dataOutput{j,i}.position = temporalPosition{i}.position(j,:);
        dataOutput{j,i}.timestamp = timestampsNewVector(j)';
    end
end

end

