function [ data_acc, data_raw, info ] = read_NDI_tracking_files( path, testrow_name)
%read_NDI_tracking_files reads from .tsv (tabulator separated values) files
%that originate from a NDI Track (orignal NDI software) measurement.
%
%INPUT
%
%path:
%Path of the .tsv files. No fileseparator in the end needed.
%
%testrow_name:
%If there are recodrings from different setups in 'path', you can specify
%the first part of the common filename here.
%
%OUTPUT
%
%data_acc:
%Cell with dimensions {numPositions,numSensors}. Each cell object is a
%struct with fields .position(1x3) and .orientation(1x4).
%
%data_raw:
%Cell with dimensions {numPositions,numSensors}. Each cell object is a
%struct with fields .position(100x3), .orientation(100x4), .state(100x1,
%char), .indicator(100x1).
%
%info:
%We could add information about the variable 'good_indices' of each
%position here. Also, Information about Standard Deviation could be useful.
%
%Author: Felix Achilles

    data_raw=[];
    info = [];
    files = dir([path filesep testrow_name '*.tsv']);
    names = sort({files(:).name});    
    
    
    %each file should represent one position
    numPositions = numel(names);
    delimiter = '\t';
    format = '%d8';
    
    data_acc = cell(numPositions,1);
    
    if (~isempty(names))
        for i = 1:numPositions
            
            
            filename = [path filesep names{i}];
            fileIDOT = fopen(filename,'r');
            
            data_raw = textscan(fileIDOT, format, 1, 'Headerlines', 1);
            
            numSensors = data_raw{1}(1);
            data_acc_temp = cell(1,numSensors);
            
            for snum = 1:numSensors
                format = [format '%s%d%d8%s%f%f%f%f%f%f%f%f%d'];
            end
            % put readout on position 0 again
            fileIDOT = fopen(filename,'r');
            data_raw = textscan(fileIDOT, format, 'Delimiter', delimiter, 'ReturnOnError', false, 'Headerlines', 1);
            
            for snum = 1:numSensors
                offset = (snum-1)*13;
                
                % x,y,z position
                for pos=1:3
                    data_acc_temp{snum}.position(:,pos)    = data_raw{1,offset+pos+9};
                end
                
                % FOUR element quaternion, rounded, normalization necessary
                for quat=1:4
                    data_acc_temp{snum}.orientation(:,quat) = data_raw{1,offset+quat+5};
                end
                
                % possible error state, string 'OK' means okay
                data_acc_temp{snum}.state       = data_raw{1,offset+5};
                
                % quality measure between 0 and 9.9, all below 1.0 is okay
                data_acc_temp{snum}.indicator   = data_raw{1,offset+13};
                
                % find good positions
                [good_indices] = find((data_acc_temp{snum}.indicator < 1) & strcmp(data_acc_temp{snum}.state, 'OK'));
                
                % only take those lines that are good, average them, proceed
                data_acc{i,snum}.position = mean(data_acc_temp{snum}.position(good_indices,:),1);
                data_acc{i,snum}.orientation = mean(data_acc_temp{snum}.orientation(good_indices,:),1);          
            end
        end
    end
        
end

