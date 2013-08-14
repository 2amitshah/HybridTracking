function [H_X_to_XBase_interp, H_XBase_to_X_interp] = frame_interpolation(raw_Xdata, interval, frequency, quaternion_style, TSOption)
%pls do this for each sensor individually

if ~exist('quaternion_style','var')
    error('frame_interpolation::inputs - you forgot quaternion_style')
end

if ~exist(TSOption, 'var') || isempty(TSOption)
    TSOption = 'network';
end

switch quaternion_style
    case 'cpp'
        quaternion_style_td_to_m = 'CppCodeQuat';
    case 'ndi'
        quaternion_style_td_to_m = 'NDIQuat';
end
if isempty(interval)
    
if strcmp(TSOption, 'network')
    begin_TS_ns = raw_Xdata{1}.TimeStamp;% nanoseconds
elseif strcmp(TSOption, 'device')
    begin_TS_ns = raw_Xdata{1}.DeviceTimeStamp;% seconds
end
    
    end_TS_ns = raw_Xdata{end}.TimeStamp;
else
    begin_TS_ns = interval(1);
    end_TS_ns = interval(2);
end

step = round(1e9/frequency);
timestampsNewVector = begin_TS_ns:step:end_TS_ns;
new_TS_bool = false(size(timestampsNewVector));
indices_relating_new_to_old_TS = zeros(size(timestampsNewVector));

[H_X_to_XBase_cell, H_XBase_to_X_cell] = trackingdata_to_matrices(raw_Xdata, quaternion_style_td_to_m);
H_X_to_XBase = H_X_to_XBase_cell{1};
H_XBase_to_X = H_XBase_to_X_cell{1};

% determine valid indices
% raw_Xdata_arraystruct = [raw_Xdata{:}];
numRawPts = size(raw_Xdata, 1);
raw_Xdata_valid_array = zeros(numRawPts,1);
raw_Xdata_timestamps = zeros(numRawPts,1);
for i = 1:numRawPts
    if ~isempty(raw_Xdata{i})
    raw_Xdata_valid_array(i) = raw_Xdata{i}.valid;
    raw_Xdata_timestamps(i) = raw_Xdata{i}.TimeStamp;
    end
end
% raw_Xdata_valid_array = [raw_Xdata_arraystruct.valid];
raw_Xdata_valid_indices = find(raw_Xdata_valid_array);
if isempty(raw_Xdata_valid_indices)
    error('dafuq?')
end

% restrict timestampsNewVector to intervals between two neighboring valid points
% raw_Xdata_timestamps = [raw_Xdata_arraystruct.TimeStamp];
numValid = numel(raw_Xdata_valid_indices);
for i = 1:numValid-1
    % if the next valid index is a direct follower
    if raw_Xdata_valid_indices(i+1) == raw_Xdata_valid_indices(i)+1
        
        % mark all new Timestamps inbetween the two as true
        temp_range_of_new_TS_bool = ...
            [ timestampsNewVector > raw_Xdata_timestamps(raw_Xdata_valid_indices(i)) ] &...
            [ timestampsNewVector < raw_Xdata_timestamps(raw_Xdata_valid_indices(i)+1) ];
        new_TS_bool = new_TS_bool |  temp_range_of_new_TS_bool;
        indices_relating_new_to_old_TS(temp_range_of_new_TS_bool) = i;
    end
end

numInterpPoints = numel(timestampsNewVector);
H_X_to_XBase_interp = zeros(4,4,numInterpPoints);
H_XBase_to_X_interp = zeros(4,4,numInterpPoints);

timestampsNewVector = timestampsNewVector(new_TS_bool);
% timestampsNewVectorToFill = zeros(size(timestampsNewVector));
% timestampsNewVectorToFill(new_TS_bool) = timestampsNewVector(new_TS_bool);

indices_relating_new_to_old_TS = indices_relating_new_to_old_TS(new_TS_bool);

% do the actual interpolation
for i = 1:numValid-1
    new_TS_indices = find(indices_relating_new_to_old_TS == i);
    if ~isempty(new_TS_indices)
        % calc ratio
        r = (timestampsNewVector(new_TS_indices(1)) - raw_Xdata_timestamps(raw_Xdata_valid_indices(i))) / (raw_Xdata_timestamps(raw_Xdata_valid_indices(i)+1) - raw_Xdata_timestamps(raw_Xdata_valid_indices(i)));
        % build difference transformation from (i) to (i+1)
        H_iplusone_to_i = H_XBase_to_X(:,:,raw_Xdata_valid_indices(i)) * H_X_to_XBase(:,:,raw_Xdata_valid_indices(i)+1);
        if r > 1000*eps
            H_step = expm(logm(H_iplusone_to_i) * r);
        else
            disp 'frame_interpolation::r interpolated point too close to original point. Original point is taken.'
            H_step = eye(4);
        end
        % to get nice homogenuous matrices, hard-code the last row
        H_step(4,:) = [0 0 0 1];
        % for some reasons, complex numbers can be created. Cut off the imaginary
        % part
        H_step = real(H_step);
        H_X_to_XBase_interp(:,:,new_TS_indices(1)) = H_X_to_XBase(:,:,raw_Xdata_valid_indices(i)) * H_step;
        H_XBase_to_X_interp(:,:,new_TS_indices(1)) = inv(H_X_to_XBase_interp(:,:,new_TS_indices(1)));
        
        if numel(new_TS_indices) > 1
        % new ratio for the rest of the interp points
        r = step / (raw_Xdata_timestamps(raw_Xdata_valid_indices(i)+1) - raw_Xdata_timestamps(raw_Xdata_valid_indices(i)));
        H_step = expm(logm(H_iplusone_to_i) * r);
        H_step(4,:) = [0 0 0 1];
        H_step = real(H_step);
        % rest of the interp points in the interval are built similar
        for j = new_TS_indices(2:end)
            H_X_to_XBase_interp(:,:,j) = H_X_to_XBase_interp(:,:,j-1) * H_step;
            H_XBase_to_X_interp(:,:,j) = inv(H_X_to_XBase_interp(:,:,j));
        end
        end
    end

end

end