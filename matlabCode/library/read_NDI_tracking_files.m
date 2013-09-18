function [ data_acc, data_raw, info ] = read_NDI_tracking_files( path, testrow_name, recording_scheme, NDIdevice)
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
%the first part of the common filename here, e.g. 'EM_straightline_'.
%
%recording_scheme:
%Can be 'static' or 'dynamic'. In 'static' case, the algorithm will average
%the readings of each input file it finds. In 'dynamic' case, it will only
%allow one input file and take each line of that file as one position
%without averaging over them.
%
%NDIdevice:
%Can be 'Aurora' or 'Polaris'. This information is important to calculate
%the correct DeviceTimeStamp as one field of the output struct. Both
%devices give out a frame number which is converted into seconds based on
%different definitions for each device. Whatever NDI thought by doing
%that...
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
%info: (not used)
%We could add information about the variable 'good_indices' of each
%position here. Also, Information about Standard Deviation could be useful.
%
%Author: Felix Achilles, August 2013

if ~exist('recording_scheme','var')
    recording_scheme = 'static';
end
if ~exist('NDIdevice','var')
    NDIdevice = 'Aurora';
end
    data_raw=[];
    info = [];
    files = dir([path filesep testrow_name '*.tsv']);
    names = sort({files(:).name});    
    
    
    %each file should represent one position
    numPositions = numel(names);
    delimiter = '\t';
    format = '%d8';
    
    if numPositions==1 && strcmp(recording_scheme,'dynamic')
                filename = [path filesep names{1}];
                fileIDOT = fopen(filename,'r');

                data_raw = textscan(fileIDOT, format, 1, 'Headerlines', 1);
                fclose(fileIDOT);
                numSensors = data_raw{1}(1);
                data_acc = cell(1,numSensors);
                data_acc_temp = cell(1,numSensors);
                good_indices = cell(1,numSensors);
                
                for snum = 1:numSensors
                    format = [format '%s%d%d8%s%f%f%f%f%f%f%f%f%d'];
                end
                % put readout on position 0 again
                fileIDOT = fopen(filename,'r');
                data_raw = textscan(fileIDOT, format, 'Delimiter', delimiter, 'ReturnOnError', false, 'Headerlines', 1);
                fclose(fileIDOT);
                
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
                    if strcmp(NDIdevice, 'Polaris')
                        data_acc_temp{snum}.DeviceTimeStamp  = double(data_raw{1,offset+3}) / 60;
                    elseif strcmp(NDIdevice, 'Aurora')
                        data_acc_temp{snum}.DeviceTimeStamp  = double(data_raw{1,offset+3}) / (8*40);
                    else
                        error('wrong device name in variable #NDIdevice')
                    end
                    % possible error state, string 'OK' means okay
                    data_acc_temp{snum}.state       = data_raw{1,offset+5};

                    % quality measure between 0 and 9.9, all below 1.0 is okay
                    data_acc_temp{snum}.indicator   = data_raw{1,offset+13};

                    % find good positions
                    good_indices{snum} = find((data_acc_temp{snum}.indicator < 1) & strcmp(data_acc_temp{snum}.state, 'OK'));

                    % only take those lines that are good, average them, proceed
                    data_acc{1,snum}.position = data_acc_temp{snum}.position(good_indices{snum},:);
                    data_acc{1,snum}.orientation = data_acc_temp{snum}.orientation(good_indices{snum},:);          
                    data_acc{1,snum}.DeviceTimeStamp = data_acc_temp{snum}.DeviceTimeStamp;
                end
                % change to desired output format
                numPts = size(data_acc_temp{1}.position, 1);
                data_acc_temp = cell(numPts,numSensors);
                for j = 1:numSensors
                    for i = 1:numPts
                        if any(i == good_indices{j})
                            data_acc_temp{i,j}.position = data_acc{1,j}.position(i,:);
                            data_acc_temp{i,j}.orientation = data_acc{1,j}.orientation(i,:);
                            data_acc_temp{i,j}.valid = 1;
                        else
                            data_acc_temp{i,j}.position = [0 0 0];
                            data_acc_temp{i,j}.orientation = [0 0 0 0];
                            data_acc_temp{i,j}.valid = 0;
                        end
                        data_acc_temp{i,j}.DeviceTimeStamp = data_acc{1,j}.DeviceTimeStamp(i);
                    end
                end
                data_acc = data_acc_temp;
    else
        
        if (~isempty(names))
            


                filename = [path filesep names{1}];
                fileIDOT = fopen(filename,'r');

                data_raw = textscan(fileIDOT, format, 1, 'Headerlines', 1);
                fclose(fileIDOT);
                numSensors = data_raw{1}(1);
                data_acc = cell(numPositions,numSensors); %TODO
            for i = 1:numPositions
                data_acc_temp = cell(1,numSensors);
                
                filename = [path filesep names{i}];
                fileIDOT = fopen(filename,'r');

                data_raw = textscan(fileIDOT, format, 1, 'Headerlines', 1);
                fclose(fileIDOT);
                
                for snum = 1:numSensors
                    format = [format '%s%d%d8%s%f%f%f%f%f%f%f%f%d'];
                end
                % put readout on position 0 again
                filename = [path filesep names{i}];
                fileIDOT = fopen(filename,'r');
                data_raw = textscan(fileIDOT, format, 'Delimiter', delimiter, 'ReturnOnError', false, 'Headerlines', 1);
                fclose(fileIDOT);
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
        
end

