% this function is for combining the data of the two sensors. This means that
% the data of OT and EM is put into a Kalman filter as soon as it is
% available. Depending on the system from which the data is coming, the
% R-matrix (Measurement noise) is set accordingly.

function KalmanData = kalman_fusion_position_orientation_DeviceTS(path, kalmanfrequencyHz, verbosity)
%% read arguments, set defaults
filenames_struct = path;
if isstruct(filenames_struct)
    testrow_name_EM = filenames_struct.EMfiles;
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

%% read in raw data
[data_OT_tmp, data_EMT_tmp] = read_Direct_NDI_PolarisAndAurora(filenames_struct, 'vRelease');
%todo: compute the transformation between the different sensors of EM..
% so far: delete the second sensor :)
data_EM_Sensor1 = data_EMT_tmp(1:size(data_EMT_tmp,1),1);

%% perform synchronization
EM_minus_OT_offset = sync_from_file(filenames_struct, 'vRelease', 'device');
numPtsEMT = size(data_EM_Sensor1,1);
for i = 1:numPtsEMT
    if ~isempty(data_EM_Sensor1{i})
        % move EM timestamps into timeframe of Optical (because Optical is our common reference)
        data_EM_Sensor1{i}.DeviceTimeStamp = data_EM_Sensor1{i}.DeviceTimeStamp - EM_minus_OT_offset;
    end
end

%% options to simulate sensor failures
% deleteEMmin = 50;
% deleteEMmax = 70;
% deleteOTmin = 200;
% deleteOTmax = 210;
% deleteBothmin = 150;
% deleteBothmax = 180;

%% determine earliest and latest common timestamp
interval = obtain_boundaries_for_interpolation(data_OT_tmp, data_EM_Sensor1, 'device');
%startTime = interval(1);
startTime = data_OT_tmp{3}.DeviceTimeStamp;
endTime = interval(2);

%% get Y, ( = H_OCS_to_EMCS)
load('H_OT_to_EMT.mat');
[Y,~] = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease','device');

%% compute homogenuous matrices from struct data
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM_Sensor1, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT_tmp, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};

%% calculate OT position using EMT position
H_OT_to_EMCS_by_EMT = zeros(4,4,numPtsEMT);
data_OT_to_EMCS_by_EMT = cell(numPtsEMT,1);

for i = 1:numPtsEMT
    H_OT_to_EMCS_by_EMT(:,:,i) = H_EMT_to_EMCS(:,:,i) * H_OT_to_EMT;
    if(~isempty(data_EM_Sensor1{i}) && data_EM_Sensor1{i}.valid == 1) % && data_EM_common{i}.valid == 1) %DEBUG: here we limit data_EM_common_by_OT{i}.valid to only be valid when OT AND EMT at that time are valid.  
        data_OT_to_EMCS_by_EMT{i}.DeviceTimeStamp = data_EM_Sensor1{i}.DeviceTimeStamp;      
        data_OT_to_EMCS_by_EMT{i}.position = (H_OT_to_EMCS_by_EMT(1:3,4,i))';
        data_OT_to_EMCS_by_EMT{i}.orientation = [(rot2quat(H_OT_to_EMCS_by_EMT(1:3, 1:3, i)))' 1];   
        data_OT_to_EMCS_by_EMT{i}.valid = data_EM_Sensor1{i}.valid;
    else
        data_OT_to_EMCS_by_EMT{i}.valid = 0;
    end
end

%% calculate OT position in EMCS frame
numPtsOT = size(data_OT_tmp,1);
H_OT_to_EMCS = zeros(4,4,numPtsOT);
data_OT_to_EMCS = cell(numPtsOT,1);

for i = 1:numPtsOT
    H_OT_to_EMCS(:,:,i) = Y*H_OT_to_OCS(:,:,i);
    if(~isempty(data_OT_tmp{i}) && data_OT_tmp{i}.valid == 1) % && data_EM_common{i}.valid == 1) %DEBUG: here we limit data_EM_common_by_OT{i}.valid to only be valid when OT AND EMT at that time are valid.  
        data_OT_to_EMCS{i}.DeviceTimeStamp = data_OT_tmp{i}.DeviceTimeStamp;      
        data_OT_to_EMCS{i}.position = (H_OT_to_EMCS(1:3,4,i))';
        data_OT_to_EMCS{i}.orientation = [(rot2quat(H_OT_to_EMCS(1:3, 1:3, i)))' 1];   
        data_OT_to_EMCS{i}.valid = data_OT_tmp{i}.valid;
    else
        data_OT_to_EMCS{i}.valid = 0;
    end
end

%% initialize matrices and vectors for Kalman

% Kalman update timestep
timestep_in_s = 1 / kalmanfrequencyHz; % * 10^9;

% initial state vector init_x, start with timestep 3
% (in order to have the first and second derivative)
timestep23 = (data_OT_to_EMCS{3}.DeviceTimeStamp - data_OT_to_EMCS{2}.DeviceTimeStamp);
x_dot = (data_OT_to_EMCS{3}.position(1) - data_OT_to_EMCS{2}.position(1))/timestep23;
y_dot = (data_OT_to_EMCS{3}.position(2) - data_OT_to_EMCS{2}.position(2))/timestep23;
z_dot = (data_OT_to_EMCS{3}.position(3) - data_OT_to_EMCS{2}.position(3))/timestep23;
% convert quaternion to explicit XYZ-Euler
[x_angle, y_angle, z_angle] = quat2angle( [data_OT_to_EMCS{2}.orientation(4) data_OT_to_EMCS{2}.orientation(1) data_OT_to_EMCS{2}.orientation(2) data_OT_to_EMCS{2}.orientation(3) ;...
            [data_OT_to_EMCS{3}.orientation(4) data_OT_to_EMCS{3}.orientation(1) data_OT_to_EMCS{3}.orientation(2) data_OT_to_EMCS{3}.orientation(3) ]], 'XYZ');
x_angvel = (x_angle(2) - x_angle(1))/timestep23;
y_angvel = (y_angle(2) - y_angle(1))/timestep23;
z_angvel = (z_angle(2) - z_angle(1))/timestep23;
% TODO check for singularities, difference between -90 and +90 degrees
% would result in a huge angular velocity, for now: output and check by
% user
disp('calculated angular velocities. everything in normal range?')
disp([x_angvel y_angvel z_angvel])
disp('###############################')
% state vector without acceleration but with attitude (q) and angular
% velocity
initx = [
        data_OT_to_EMCS{3}.position(1);
        data_OT_to_EMCS{3}.position(2);
        data_OT_to_EMCS{3}.position(3);
        x_dot;
        y_dot;
        z_dot;
        data_OT_to_EMCS{3}.orientation(1);
        data_OT_to_EMCS{3}.orientation(2);
        data_OT_to_EMCS{3}.orientation(3);
        data_OT_to_EMCS{3}.orientation(4);
        x_angvel;
        y_angvel;
        z_angvel
        ];

x = initx;

statesize = 13;
observationsize = 13;

% state transition matrix A
A = eye(statesize);
for i = 1:3 %6
    A(i,i+3) = timestep_in_s;
end
disp('A for constant velocity case')
disp(A)

H = zeros(observationsize,statesize);
H(1:observationsize, 1:observationsize) = eye(observationsize);
disp('H for constant velocity case')
disp(H)

%initial state covariance (P)
initP = .05 * eye(statesize);
initP(4:6,4:6) = 0.1 * eye(3);
% initP(7:9,7:9) = 1400 * eye(3);
P = initP;

% process noise covariance matrix Q
Q = 0.5 * eye(statesize);
Q(4:6,4:6) = 0.5 * kalmanfrequencyHz * 2 * eye(3);
% Q(7:9,7:9) = 100 * eye(3);

% measurement noise covariance matrix R
position_variance_OT = 0.5;
position_variance_EM = 1;
R_OT = position_variance_OT*eye(observationsize); %the higher the value, the less the measurement is trusted
R_OT(4:6,4:6) =  2 * position_variance_OT * kalmanfrequencyHz * eye(3);
R_EM = position_variance_EM*eye(observationsize);
R_EM(4:6,4:6) =  2 * position_variance_EM * kalmanfrequencyHz * eye(3);

%% perfrom Kalman filtering, take whatever is available (OT or EM) and feed it into the Kalman filter
t = startTime;
indexOT = 4;
indexEM = 1;
dataind = 1;
% create a dataset of OT and EM points, sorted by synchronized timestamp
sortedData = cell(numPtsEMT + numPtsOT - 3, 1);
while(dataind < numPtsEMT + numPtsOT - 3 && indexOT <= numPtsOT && indexEM <= numPtsEMT )
    if(data_OT_to_EMCS{indexOT}.valid && data_OT_to_EMCS_by_EMT{indexEM}.valid)
        if(data_OT_to_EMCS{indexOT}.DeviceTimeStamp < data_OT_to_EMCS_by_EMT{indexEM}.DeviceTimeStamp)
           sortedData{dataind,1} = data_OT_to_EMCS{indexOT};
           sortedData{dataind}.fromOT = 1;
           indexOT = indexOT + 1;
        else
           sortedData{dataind,1} = data_OT_to_EMCS_by_EMT{indexEM};
           sortedData{dataind}.fromOT = 0;
           indexEM = indexEM + 1;
        end    
        dataind = dataind+1;
    else
        if ~(data_OT_to_EMCS{indexOT}.valid)
            indexOT = indexOT + 1;
        elseif ~data_OT_to_EMCS_by_EMT{indexEM}.valid
            indexEM = indexEM + 1;
        end
    end
end
dataind = 1;
index = 0; %index of the last used measurement

KalmanData = cell(numel(startTime:timestep_in_s:endTime), 1);
while(t < endTime)
    oldIndex = index;
    % count how many measurements came in since the last Kalman step
    while(index < size(sortedData,1) && ~isempty(sortedData{index+1}) && sortedData{index+1}.DeviceTimeStamp < t)
        index = index + 1;
    end
    % update velocity for measurement update step
    if (dataind > 2)
        x_dot = (KalmanData{dataind-1}.position(1) - KalmanData{dataind-2}.position(1)) / timestep_in_s;
        y_dot = (KalmanData{dataind-1}.position(2) - KalmanData{dataind-2}.position(2)) / timestep_in_s;
        z_dot = (KalmanData{dataind-1}.position(3) - KalmanData{dataind-2}.position(3)) / timestep_in_s;
    end
    %use all measurements of sorted data, starting with oldIndex+1 until
    %index
    if(oldIndex~=index) % if there were new measurements since the last update
        for i = oldIndex+1:index
            if (sortedData{i}.fromOT) %use OT
                R = R_OT;
            else
                R = R_EM;
            end
            
            if(i==(oldIndex+1))
                currentTimestep = (sortedData{i}.DeviceTimeStamp - (t-timestep_in_s));
            else
                currentTimestep = (sortedData{i}.DeviceTimeStamp - sortedData{i-1}.DeviceTimeStamp);
            end
            % build A
            A = eye(statesize);
            for j = 1:3 %6 
                A(j,j+3) = currentTimestep;
            end
            % state prediction
            x_minus = A * x;
            P_minus = A * P * A' + Q; 
            % state update (correction by measurement)
            K = P_minus * H' * ((H * P_minus * H' + R)^-1); %Kalman gain
            z = [ sortedData{i}.position(1); sortedData{i}.position(2); sortedData{i}.position(3); x_dot; y_dot; z_dot]; %measurement      
            x = x_minus + K * (z - (H * x_minus));
            P = (eye(statesize) - K * H ) * P_minus;                    
        end
        % compute timestep to predict until the end of the time interval
        currentTimestep = (t - sortedData{index}.DeviceTimeStamp);        
        
    else %compute timestep for prediction
        currentTimestep = timestep_in_s;
    end
    
    % build A
    A = eye(statesize);
    for j = 1:3 %6
        A(j,j+3) = currentTimestep;
    end
    % state prediction
    x_minus = A * x;
    P_minus = A * P * A' + Q;
    P = P_minus;
    x = x_minus;
    
    %put filtered data into KalmanData struct
    KalmanData{dataind,1}.position = x(1:3)';
    KalmanData{dataind,1}.speed = [x_dot; y_dot; z_dot];
    KalmanData{dataind,1}.P = P;
    KalmanData{dataind,1}.KalmanTimeStamp = t;
    
    dataind = dataind + 1;
    t = t + timestep_in_s;
end


%% plots
for i = 1:size(KalmanData,1)
    KalmanData{i}.orientation = [.5 .5 .5 .5];
end

% plot path in 3D
orig_cell = trackingdata_to_matrices(data_OT_to_EMCS, 'cpp');
fromEMT_cell = trackingdata_to_matrices(data_OT_to_EMCS_by_EMT, 'cpp');
data_cell = trackingdata_to_matrices(KalmanData, 'cpp');
datafig = Plot_points(data_cell, [], 1, 'o');
Plot_points(fromEMT_cell,datafig,2,'+');
Plot_points(orig_cell, datafig, 3, 'x');
%Plot_points(orig_cell,[], 3, 'x');
title('data EM: x, data OT: +, data filtered: o');

% plot velocities
VelocityFigure = figure;
title('Speeds [mm/s] in x, y, z direction over time in [s].')
KalmanData_structarray = [KalmanData{:}];
KalmanSpeeds = [KalmanData_structarray.speed];
KalmanTime = [KalmanData_structarray.KalmanTimeStamp];
subplot(3,1,1)
plot(KalmanTime, KalmanSpeeds(1,:), 'r')
title('x\_dot')
subplot(3,1,2)
plot(KalmanTime, KalmanSpeeds(2,:), 'g')
title('y\_dot')
subplot(3,1,3)
plot(KalmanTime, KalmanSpeeds(3,:), 'b')
title('z\_dot')

% plot development of P entries
CovarianceFigure = figure;
title('Diagonal elements of state covariance P.')
KalmanCovariance = [KalmanData_structarray.P];
KalmanCovariance = reshape(KalmanCovariance,statesize,statesize,numel(KalmanTime));
posvar = zeros(1,numel(KalmanTime));
speedvar = posvar;
for i = 1:numel(KalmanTime)
posvar(i) = norm([KalmanCovariance(1,1,i) KalmanCovariance(2,2,i) KalmanCovariance(2,2,i)]);
speedvar(i) = norm([KalmanCovariance(4,4,i) KalmanCovariance(5,5,i) KalmanCovariance(6,6,i)]);
end
subplot(2,1,1)
plot(KalmanTime, posvar, 'b')
title('position variance')
subplot(2,1,2)
plot(KalmanTime, speedvar, 'r')
title('speed variance')

clear KalmanData_structarray
end



















