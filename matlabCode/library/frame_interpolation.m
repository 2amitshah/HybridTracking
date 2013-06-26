function [H_X_to_XBase_interp, H_XBase_to_X_interp] = frame_interpolation(raw_Xdata, interval, frequency, quaternion_style)
%pls do this for each sensor individually

if ~exist('quaternion_style','var')
    error('frame_interpolation::inputs - you forgot quaternion_style')
end

switch quaternion_style
    case 'cpp'
        quaternion_style_td_to_m = 'CppCodeQuat';
    case 'ndi'
        quaternion_style_td_to_m = 'NDIQuat';
end

begin_TS_ns = interval(1);
end_TS_ns = interval(2);
step = round(1e9/frequency);
timestampsNewVector = begin_TS_ns:step:end_TS_ns;
new_TS_bool = false(size(timestampsNewVector));


% determine valid indices
raw_Xdata_arraystruct = [raw_Xdata{:}];
raw_Xdata_valid_array = [raw_Xdata_arraystruct.valid];
raw_Xdata_valid_indices = find(raw_Xdata_valid_array);
if isempty(raw_Xdata_valid_indices)
    error('dafuq?')
end

% restrict timestampsNewVector to intervals between two neighboring valid points
raw_Xdata_timestamps = [raw_Xdata_arraystruct.TimeStamp];
numValid = numel(raw_Xdata_valid_indices);
for i = 1:numValid-1
    % if the next valid index is a direct follower
    if raw_Xdata_valid_indices(i+1) == raw_Xdata_valid_indices(i)+1
        % mark all new Timestamps inbetween the two as true
        new_TS_bool = new_TS_bool |  ...
            [ timestampsNewVector > raw_Xdata_timestamps(raw_Xdata_valid_indices(i)) ] &...
            [ timestampsNewVector < raw_Xdata_timestamps(raw_Xdata_valid_indices(i)+1) ];        
    end
end

timestampsNewVector = timestampsNewVector(new_TS_bool);
numInterpPoints = numel(timestampsNewVector);

[H_X_to_XBase_cell, H_XBase_to_X_cell] = trackingdata_to_matrices(raw_Xdata, quaternion_style_td_to_m);
H_X_to_XBase = H_X_to_XBase_cell{1};
H_XBase_to_X = H_XBase_to_X_cell{1};

H_X_to_XBase_interp = zeros(4,4,numInterpPoints);
H_XBase_to_X_interp = zeros(4,4,numInterpPoints);

counter = 1;

% do the actual interpolation
for i = 1:numValid-1
    % if the next valid index is a direct follower
    if raw_Xdata_valid_indices(i+1) == raw_Xdata_valid_indices(i)+1
        
        % calc ratio
        r = (timestampsNewVector(counter) - raw_Xdata_timestamps(raw_Xdata_valid_indices(i))) / (raw_Xdata_timestamps(raw_Xdata_valid_indices(i)+1) - raw_Xdata_timestamps(raw_Xdata_valid_indices(i)));
        % build difference transformation from (i) to (i+1)
        H_iplusone_to_i = H_XBase_to_X(:,:,raw_Xdata_valid_indices(i)) * H_X_to_XBase(:,:,raw_Xdata_valid_indices(i)+1);
        if r > 10*eps
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
        H_X_to_XBase_interp(:,:,counter) = H_X_to_XBase(:,:,raw_Xdata_valid_indices(i)) * H_step;
        H_XBase_to_X_interp(:,:,counter) = inv(H_X_to_XBase_interp(:,:,counter));
        counter = counter + 1;
        
        % new ratio for the rest of the interp points
        r = step / (raw_Xdata_timestamps(raw_Xdata_valid_indices(i)+1) - raw_Xdata_timestamps(raw_Xdata_valid_indices(i)));
        H_step = expm(logm(H_iplusone_to_i) * r);
        H_step(4,:) = [0 0 0 1];
        H_step = real(H_step);
        % rest of the interp points in the interval are built similar
        while(counter <= numInterpPoints && timestampsNewVector(counter) < raw_Xdata_timestamps(raw_Xdata_valid_indices(i)+1))
            H_X_to_XBase_interp(:,:,counter) = H_X_to_XBase_interp(:,:,counter-1) * H_step;
            H_XBase_to_X_interp(:,:,counter) = inv(H_X_to_XBase_interp(:,:,counter));
            counter = counter + 1;
        end
    end
end

end