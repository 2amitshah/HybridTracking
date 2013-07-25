% this function is for combining the data of the two sensors. Tthis means that
% the data of OT and EM is put into a Kalman filter as soon as it is
% available. Depending on the system from which the data is coming, the
% R-matrix is set accordingly.

function datafiltered = kalman_fusion_positions(path, kalmanfrequencyHz, verbosity)

filenames_struct = path;
if isstruct(filenames_struct)
    testrow_name_EMT = filenames_struct.EMfiles;
    testrow_name_OT = filenames_struct.OTfiles;
    path = filenames_struct.folder;
else
    warning('GeneralWarning:pathStruct',['Please use the new filenames_struct-feature.\n'...
        ' ''path'' can now be a struct, so you don''t always have to change the default ''testrow_name_EMT'' and ''testrow_name_OT''.'])
end

if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
end
if ~exist('kalmanfrequencyHz','var')
    kalmanfrequencyHz = 10;
end
if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
end
if ~exist('testrow_name_EM', 'var')
    testrow_name_EM = 'EMTrackingcont_1';
end
if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OpticalTrackingcont_1';
end

% get data (without any interpolation
[data_OT_tmp, data_EMT_tmp, errorTimeStampsOT, errorTimeStampsEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EM);
%todo: compute the transformation between the different sensors of EM..so
%far: delete the second sensor :)

%(using static points) and transfer all the points to the position of
%sensor 1 --> result: cell with only one column
% data_EM_tmp2(1:size(data_EMT_tmp,1),1) = data_EMT_tmp(1:size(data_EMT_tmp,1),1);
data_EM_tmp2 = data_EMT_tmp(1:size(data_EMT_tmp,1),1);
idx = 1;
% deleteEMmin = 50;
% deleteEMmax = 70;
% deleteOTmin = 200;
% deleteOTmax = 210;
% deleteBothmin = 150;
% deleteBothmax = 180;

for i = 1:size(data_EM_tmp2,1)
    if(~isempty(data_EM_tmp2{i}))
        if (data_EM_tmp2{i}.valid) %&& (i <deleteEMmin || i > deleteEMmax) && (i<deleteBothmin || i>deleteBothmax)
            data_EM{idx,1} = data_EM_tmp2{i};
            %data_EM{idx,1}.TimeStamp = data_EM{idx,1}.TimeStamp + 2*10^7;
            idx = idx+1;
        end
    end
end
idx = 1;

for i = 1:size(data_OT_tmp,1)
    if(~isempty(data_OT_tmp{i}))
        if (data_OT_tmp{i}.valid) %&& (i < deleteOTmin || i > deleteOTmax) && (i<deleteBothmin || i>deleteBothmax))
            data_OT{idx,1} = data_OT_tmp{i};
            idx = idx + 1;
        end
    end
end


%% determine earliest and latest common timestamp
interval = obtain_boundaries_for_interpolation(data_OT, data_EM);
%startTime = interval(1);
startTime = data_OT{3}.TimeStamp;
endTime = interval(2);

% get Y, equal to EMCS_to_OCS
load(which('H_OT_to_EMT.mat'));
[Y,~] = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease');

%% compute EMT_by_OT data
% Relevant matrix for computing transformations
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};

%% calculate where EM tracker should be
numPtsOT = size(data_OT,1);
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPtsOT);

data_EM_by_OT = cell(numPtsOT,1);

for i = 1:numPtsOT
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT; %data_EM_common_by_OT    
    if(data_OT{i}.valid == 1) % && data_EM_common{i}.valid == 1) %DEBUG: here we limit data_EM_common_by_OT{i}.valid to only be valid when OT AND EMT at that time are valid.  
        data_EM_by_OT{i}.TimeStamp = data_OT{i}.TimeStamp;      
        data_EM_by_OT{i}.position = transpose(H_EMT_to_EMCS_by_OT(1:3,4,i));
        data_EM_by_OT{i}.orientation = transpose(rot2quat(H_EMT_to_EMCS_by_OT(1:3, 1:3, i)));   
        data_EM_by_OT{i}.valid = data_OT{i}.valid;
    else
        data_EM_by_OT{i}.valid = 0;
    end
end

%% initialize matrices and vectors
% initial state vector init_x, start with timestep 3 (in order to have the first and second
% derivative)
timestep23 = (data_EM_by_OT{3}.TimeStamp - data_EM_by_OT{2}.TimeStamp)/10^9;
timestep12 = (data_EM_by_OT{2}.TimeStamp - data_EM_by_OT{1}.TimeStamp)/10^9;
timestep13half = (data_EM_by_OT{3}.TimeStamp - data_EM_by_OT{1}.TimeStamp)/(2*10^9);
x_dot = (data_EM_by_OT{3}.position(1) - data_EM_by_OT{2}.position(1))/timestep23;
y_dot = (data_EM_by_OT{3}.position(2) - data_EM_by_OT{2}.position(2))/timestep23;
z_dot = (data_EM_by_OT{3}.position(3) - data_EM_by_OT{2}.position(3))/timestep23;
x_2dot = (x_dot - ((data_EM_by_OT{2}.position(1) - data_EM_by_OT{1}.position(1))/timestep12)) / timestep13half;
y_2dot = (y_dot - ((data_EM_by_OT{2}.position(2) - data_EM_by_OT{1}.position(2))/timestep12)) / timestep13half;
z_2dot = (z_dot - ((data_EM_by_OT{2}.position(3) - data_EM_by_OT{1}.position(3))/timestep12)) / timestep13half;

initx = [data_EM_by_OT{3}.position(1); data_EM_by_OT{3}.position(2); data_EM_by_OT{3}.position(3); x_dot; y_dot; z_dot; x_2dot; y_2dot; z_2dot ] 
x = initx;
timestep_in_s = 1 / kalmanfrequencyHz; % * 10^9;
timestep_in_nanos = timestep_in_s * 10^9;

statesize = 9;
observationsize = 3;
% state transition matrix A
A = eye(statesize);
for i = 1:6
    A(i,i+3) = timestep_in_s;
end
for i = 1:3
    A(i,i+6) = .5 * timestep_in_s^2;
end
A

H = zeros(observationsize,statesize);
H(1:observationsize, 1:observationsize) = eye(observationsize);
H

%initial state covariance (P)
initP = .25*eye(statesize);
initP(4:6,4:6) = 15 * eye(3);
initP(7:9,7:9) = 1400 * eye(3);
P = initP;
% process noise covariance matrix Q
Q = 1*eye(statesize);
Q(4:6,4:6) = 5 * eye(3);
Q(7:9,7:9) = 100 * eye(3);
% measurement noise covariance matrix R
position_variance_OT = 0.1;
position_variance_EM = 0.2;
R_OT = position_variance_OT*eye(observationsize); %the higher the value, the less the measurement is trusted
%R_OT(4:6,4:6) =  2 * position_variance_OT * 20* eye(3);
R_EM = position_variance_EM*eye(observationsize);
%R_EM(4:6,4:6) =  2 * position_variance_EM * 20* eye(3);


%% take whatever is available (OT or EM) and feed it into the Kalman filter
t = startTime;
indexOT = 4;
indexEM = 1;
dataind = 1;
numPtsEM = size(data_EM,1);
while(dataind < numPtsEM + numPtsOT - 3)
   if(data_EM_by_OT{indexOT}.TimeStamp < data_EM{indexEM}.TimeStamp)
       sortedData{dataind,1} = data_EM_by_OT{indexOT};
       sortedData{dataind}.fromOT = 1;
       indexOT = indexOT + 1;
   else
       sortedData{dataind,1} = data_EM{indexEM};
       sortedData{dataind}.fromOT = 0;
       indexEM = indexEM +1;
   end    
   dataind = dataind+1;
end
dataind = 1;
index = 0; %index of the last used measurement
while(t < endTime)
    t = t + timestep_in_nanos;
    oldIndex = index;
    while(index < size(sortedData,1) && sortedData{index+1}.TimeStamp < t)
        index = index + 1;
    end
    %use all measurements of sorted data, starting with oldIndex+1 until
    %index
    if(oldIndex~=index)
        for i = oldIndex+1:index
            if (sortedData{i}.fromOT) %use OT
                R = R_OT;
            else
                R = R_EM;
            end
            
            if(i==(oldIndex+1))
                currentTimestep = (sortedData{i}.TimeStamp - (t-timestep_in_nanos))/(10^9);
            else
                currentTimestep = (sortedData{i}.TimeStamp - sortedData{i-1}.TimeStamp)/(10^9);
            end
            
            A = eye(statesize);
            for j = 1:6
                A(j,j+3) = currentTimestep;
            end
            for j = 1:3
                A(j,j+6) = .5 * currentTimestep^2;
            end
            %% time update (prediction)
            x_minus = A * x;
            P_minus = A * P * A' + Q; 
            %% measurement update (correction)
            if (i > 2)
                x_dot = (sortedData{i-1}.position(1) - sortedData{i-2}.position(1)) / ( (sortedData{i-1}.TimeStamp - sortedData{i-2}.TimeStamp) /10^9);
                y_dot = (sortedData{i-1}.position(2) - sortedData{i-2}.position(2)) / ( (sortedData{i-1}.TimeStamp - sortedData{i-2}.TimeStamp) /10^9);
                z_dot = (sortedData{i-1}.position(3) - sortedData{i-2}.position(3)) / ( (sortedData{i-1}.TimeStamp - sortedData{i-2}.TimeStamp) /10^9);
            end
            
            K = P_minus * H' * ((H * P_minus * H' + R)^-1); %Kalman gain
            z = [ sortedData{i}.position(1); sortedData{i}.position(2); sortedData{i}.position(3)];%; x_dot; y_dot; z_dot]; %measurement      
            x = x_minus + K * (z - (H * x_minus));
            P = (eye(statesize) - K * H ) * P_minus;                    
        end
        % compute timestep to predict until the end of the time interval
        currentTimestep = (t - sortedData{index}.TimeStamp)/10^9;        
        
    else %compute timestep for only prediction
        currentTimestep = timestep_in_nanos/10^9;
    end
    
    %prediction
    A = eye(statesize);
    for j = 1:6
        A(j,j+3) = currentTimestep;
    end
    for j = 1:3
        A(j,j+6) = .5 * currentTimestep^2;
    end
    %% time update (prediction)
    x_minus = A * x;
    P_minus = A * P * A' + Q; 
    x = x_minus;
    %put filtered data into datafiltered
    data{dataind,1}.position = x(1:3)';
    dataind = dataind + 1;
    
end
    
for i = 1:size(data,1)
    data{i}.orientation = [.5 .5 .5 .5];
end

orig_cell = trackingdata_to_matrices(data_EM, 'cpp');
fromOT_cell = trackingdata_to_matrices(data_EM_by_OT, 'cpp');
data_cell = trackingdata_to_matrices(data, 'cpp');
datafig = Plot_points(data_cell, [], 1, 'o');
Plot_points(fromOT_cell,datafig,2,'+');
Plot_points(orig_cell, datafig, 3, 'x');
%Plot_points(orig_cell,[], 3, 'x');
title('data EM: x, data OT: +, data filtered: o');


end



















