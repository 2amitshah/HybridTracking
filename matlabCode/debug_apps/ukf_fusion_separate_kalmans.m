% This is a debug implementation of the Unscented Kalman Filter.
% There is deterministic sampling of sigma points and then a transform
% to compute mean and covariance of the forecasted values.
% This is for statistical linear motion.
% This function is for combining the data of the two sensors. This means that
% the data of OT and EM is put into a Kalman filter as soon as it is
% available. Depending on the system from which the data is coming, the
% R-matrix (Measurement noise) is set accordingly.

function KalmanData = ukf_fusion_separate_kalmans(filenames_struct, kalmanfrequencyHz, verbosity)
%% read arguments, set defaults
KF = 0; % 0: use UKF algorithm, 1: use simple Kalman algorithm

EMCSspace = 0; % 1: map everything into EMCS coordinate system, 0: map everything into OCS coordinate system.

% 'Inherent': do not create a virtual velocity measurement,
% 'LatestMeasuredData': create a velocity measurement by differencing the two latest measured data points of one modality
% 'LatestKalmanData': create a velocity measurement by differencing the two
% latest filtered points of the Kalman Output.
velocityUpdateScheme = 'LatestMeasuredData';

estimateOrientation = 0; % 0: do not estimae Orientation, only Position, 1: also estimate Orientation (nonlinearly)

% 'Inherent': do not create a virtual angular velocity measurement,
% 'LatestMeasuredData': create a angular velocity measurement by differencing the two latest measured data points of one modality
% 'LatestKalmanData': create a angular velocity measurement by differencing the two
% latest filtered points of the Kalman Output.
angvelUpdateScheme = 'Inherent';


if exist('filenames_struct', 'var') && isstruct(filenames_struct)
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
    kalmanfrequencyHz = 40;
end
if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(which(mfilename))));
    path = [pathGeneral filesep 'measurements' filesep '08.16_Measurements'];
    filenames_struct.folder = path;
end
if ~exist('testrow_name_EM', 'var')
    testrow_name_EM = 'EMT_Direct_2013_08_16_15_28_44';
    filenames_struct.EMfiles = testrow_name_EM;
end
if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OPT_Direct_2013_08_16_15_28_44';
    filenames_struct.OTfiles = testrow_name_OT;
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
[Y,YError] = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease','device');

%% compute homogenuous matrices from struct data
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM_Sensor1, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT_tmp, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};

%% calculate OT position in EMCS frame

% OT
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

% EMT
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

%% calculate OT position in OCS frame
% OT
data_OT_to_OCS = data_OT_tmp;

% EMT
H_OT_to_OCS_by_EMT = zeros(4,4,numPtsEMT);
data_OT_to_OCS_by_EMT = cell(numPtsEMT,1);

for i = 1:numPtsEMT
    H_OT_to_OCS_by_EMT(:,:,i) = Y \ H_EMT_to_EMCS(:,:,i) * H_OT_to_EMT;
    if(~isempty(data_EM_Sensor1{i}) && data_EM_Sensor1{i}.valid == 1) % && data_EM_common{i}.valid == 1) %DEBUG: here we limit data_EM_common_by_OT{i}.valid to only be valid when OT AND EMT at that time are valid.  
        data_OT_to_OCS_by_EMT{i}.DeviceTimeStamp = data_EM_Sensor1{i}.DeviceTimeStamp;      
        data_OT_to_OCS_by_EMT{i}.position = (H_OT_to_OCS_by_EMT(1:3,4,i))';
        data_OT_to_OCS_by_EMT{i}.orientation = [(rot2quat(H_OT_to_OCS_by_EMT(1:3, 1:3, i)))' 1];   
        data_OT_to_OCS_by_EMT{i}.valid = data_EM_Sensor1{i}.valid;
    else
        data_OT_to_OCS_by_EMT{i}.valid = 0;
    end
end

% choose in which CS the filtering should take place
if EMCSspace == 1
    data_OT = data_OT_to_EMCS;
    data_EMT = data_OT_to_EMCS_by_EMT;
elseif EMCSspace == 0
    data_OT = data_OT_to_OCS;
    data_EMT = data_OT_to_OCS_by_EMT;
else
    error('EMCSspace must be 0 (=use OCS space) or 1 (=use EMCS space)')
end

%% initialize matrices and vectors for Kalman

% Kalman update timestep
timestep_in_s = 1 / kalmanfrequencyHz; % * 10^9;

% initial state vector init_x, start with timestep 3
% (in order to have the first and second derivative)
timestep23 = (data_OT{3}.DeviceTimeStamp - data_OT{2}.DeviceTimeStamp);
x_dot = (data_OT{3}.position(1) - data_OT{2}.position(1))/timestep23;
y_dot = (data_OT{3}.position(2) - data_OT{2}.position(2))/timestep23;
z_dot = (data_OT{3}.position(3) - data_OT{2}.position(3))/timestep23;

x_dot_OT = x_dot;
y_dot_OT = y_dot;
z_dot_OT = z_dot;

x_dot_EM = x_dot;
y_dot_EM = y_dot;
z_dot_EM = z_dot;

% convert quaternion to explicit XYZ-Euler
[x_angle, y_angle, z_angle] = quat2angle( [data_OT{2}.orientation(4) data_OT{2}.orientation(1) data_OT{2}.orientation(2) data_OT{2}.orientation(3) ;...
            [data_OT{3}.orientation(4) data_OT{3}.orientation(1) data_OT{3}.orientation(2) data_OT{3}.orientation(3) ]], 'XYZ');
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
        data_OT{3}.position(1);
        data_OT{3}.position(2);
        data_OT{3}.position(3);
        x_dot;
        y_dot;
        z_dot;
        data_OT{3}.orientation(1);
        data_OT{3}.orientation(2);
        data_OT{3}.orientation(3);
        data_OT{3}.orientation(4);
        x_angvel;
        y_angvel;
        z_angvel
        ];

x = initx;
x_EM = x;
x_OT = x;

statesize = numel(x);
if(strcmp(velocityUpdateScheme, 'Inherent'))
    observationsize = 3;
else
    observationsize = 6;
end

% state transition matrix A
A = eye(statesize);
for i = 1:3 %6
    A(i,i+3) = timestep_in_s;
end
disp('A for constant velocity case')
disp(A)
A_EM = A;
A_OT = A;

H = zeros(observationsize,statesize);
H(1:observationsize, 1:observationsize) = eye(observationsize);
disp('H for constant velocity case')
disp(H)
H_EM = H;
H_OT = H;

%initial state covariance (P)
initP = .05 * eye(statesize);
initP(4:6,4:6) = 0.1 * eye(3);
% initP(7:9,7:9) = 1400 * eye(3);
P = initP;
P_EM = P;
P_OT = P;

% process noise covariance matrix Q
Q = 0.5 * eye(statesize);
% if ~(strcmp(velocityUpdateScheme, 'Inherent'))
    Q(4:6,4:6) = 0.5 * kalmanfrequencyHz * 2 * eye(3);
% end
% Q(7:9,7:9) = 100 * eye(3);
Q_EM = Q;
Q_OT = Q;

% measurement noise covariance matrix R
XError = 1; % error remaining from the calibration
if EMCSspace == 1
    position_variance_OT = (0.25 + YError)^2;
    position_variance_EM = (1 + XError)^2;
else
    position_variance_OT = (0.25)^2;
    position_variance_EM = (1 + XError + YError)^2;
end
R_OT = position_variance_OT*eye(observationsize); %the higher the value, the less the measurement is trusted
if ~(strcmp(velocityUpdateScheme, 'Inherent'))
    R_OT(4:6,4:6) =  2 * position_variance_OT * kalmanfrequencyHz * eye(3);
end
R_EM = position_variance_EM*eye(observationsize);
if ~(strcmp(velocityUpdateScheme, 'Inherent'))
    R_EM(4:6,4:6) =  2 * position_variance_EM * kalmanfrequencyHz * eye(3);
end

%% perform Kalman filtering, take whatever is available (OT or EM) and feed it into the Kalman filter

% indexOT = 4;
% indexEM = 1;
% dataind = 1;
% % create a dataset of OT and EM points, sorted by synchronized timestamp
% sortedData = cell(numPtsEMT + numPtsOT - 3, 1);
% while(dataind < numPtsEMT + numPtsOT - 3 && indexOT <= numPtsOT && indexEM <= numPtsEMT )
%     if(data_OT{indexOT}.valid && data_EMT{indexEM}.valid)
%         if(data_OT{indexOT}.DeviceTimeStamp < data_EMT{indexEM}.DeviceTimeStamp)
%            sortedData{dataind,1} = data_OT{indexOT};
%            sortedData{dataind}.fromOT = 1;
%            indexOT = indexOT + 1;
%         else
%            sortedData{dataind,1} = data_EMT{indexEM};
%            sortedData{dataind}.fromOT = 0;
%            indexEM = indexEM + 1;
%         end    
%         dataind = dataind+1;
%     else
%         if ~(data_OT{indexOT}.valid)
%             indexOT = indexOT + 1;
%         elseif ~data_EMT{indexEM}.valid
%             indexEM = indexEM + 1;
%         end
%     end
% end

%% start Filter
% index = 0; %index of the last used measurement of sorted data

RawDataOT_ind = 0;
RawDataEM_ind = 0;

KDataOT_ind = 1;
KDataEM_ind = 1;

KalmanDataOT = cell(numel(startTime:timestep_in_s:endTime), 1);
KalmanDataEM = KalmanDataOT;

latestOTData = data_OT{4};
latestEMData = data_EMT{1};

OnlyPredictionOT = false;
OnlyPredictionEM = false;

t = startTime;

while(t <= endTime + 1000*eps)
%     oldIndex = index;

    oldRawDataOT_ind = RawDataOT_ind;
    oldRawDataEM_ind = RawDataEM_ind;
    
    % count how many measurements came in since the last Kalman step
    % OT
    while(RawDataOT_ind < size(data_OT,1) && ~isempty(data_OT{RawDataOT_ind+1}) && data_OT{RawDataOT_ind+1}.DeviceTimeStamp < t)
        RawDataOT_ind = RawDataOT_ind + 1;
    end
    % EMT
    while(RawDataEM_ind < size(data_EMT,1) && ~isempty(data_EMT{RawDataEM_ind+1}) && data_EMT{RawDataEM_ind+1}.DeviceTimeStamp < t)
        RawDataEM_ind = RawDataEM_ind + 1;
    end
    
    % update velocity for measurement update step
    % OT
    if (KDataOT_ind > 2)
        x_dot_OT = (KalmanDataOT{KDataOT_ind-1}.position(1) - KalmanDataOT{KDataOT_ind-2}.position(1)) / timestep_in_s;
        y_dot_OT = (KalmanDataOT{KDataOT_ind-1}.position(2) - KalmanDataOT{KDataOT_ind-2}.position(2)) / timestep_in_s;
        z_dot_OT = (KalmanDataOT{KDataOT_ind-1}.position(3) - KalmanDataOT{KDataOT_ind-2}.position(3)) / timestep_in_s;
    end
    
    %use all measurements of incoming data, loop over oldIndex+1 until
    %index
    % OT
    if(oldRawDataOT_ind~=RawDataOT_ind) % if there were new measurements since the last update
        OnlyPredictionOT = false;
        for i = oldRawDataOT_ind+1:RawDataOT_ind
            % update velocity
            if strcmp(velocityUpdateScheme, 'LatestMeasuredData')
                diffTime = data_OT{i}.DeviceTimeStamp - latestOTData.DeviceTimeStamp;
                if    diffTime < 0.1 ...% maximal one reading was dropped inbetween (assuming 20Hz rate)
                   && diffTime > 1000*eps 
                    
                    x_dot_OT = (data_OT{i}.position(1) - latestOTData.position(1)) / diffTime;
                    y_dot_OT = (data_OT{i}.position(2) - latestOTData.position(2)) / diffTime;
                    z_dot_OT = (data_OT{i}.position(3) - latestOTData.position(3)) / diffTime;
                else
                    warning(['Kalman Position Data (Optical) is taken to calculate velocity at time: ' num2str(data_OT{i}.DeviceTimeStamp) 's. (DataOT index number: ' num2str(i) ')'])
                end
                z_OT = [ data_OT{i}.position(1); data_OT{i}.position(2); data_OT{i}.position(3); x_dot_OT; y_dot_OT; z_dot_OT]; %measurement      
            end
            
            if strcmp(velocityUpdateScheme, 'Inherent')
                z_OT = [ data_OT{i}.position(1); data_OT{i}.position(2); data_OT{i}.position(3)];
            end
            
            if(i==(oldRawDataOT_ind+1))
                currentTimestep = (data_OT{i}.DeviceTimeStamp - (t-timestep_in_s));
            else
                currentTimestep = (data_OT{i}.DeviceTimeStamp - data_OT{i-1}.DeviceTimeStamp);
            end
            % build A
            A_OT = eye(statesize);
            for j = 1:3 %6 
                A_OT(j,j+3) = currentTimestep;
            end
                   
            %% Kalman algorithm (KF, EKF, UKF, CKF, ...)
                       
            if (KF==1)
                %KF
                % state prediction
                x_minus_OT = A_OT * x_OT;
                P_minus_OT = A_OT * P_OT * A_OT' + Q_OT; 
                % state update (correction by measurement)
                K_OT = (P_minus_OT * H_OT') / (H_OT * P_minus_OT * H_OT' + R_OT); %Kalman gain
                x_OT = x_minus_OT + K_OT * (z_OT - (H_OT * x_minus_OT));
                P_OT = (eye(statesize) - K_OT * H_OT ) * P_minus_OT;
            else
                %UKF
                fstate_OT=@(x)(x + [currentTimestep * x(4); currentTimestep * x(5); currentTimestep * x(6); zeros(statesize-3,1)]);  % nonlinear state equations
                if strcmp(velocityUpdateScheme, 'Inherent')
                    hmeas_OT=@(x)[x(1);x(2);x(3)]; % measurement equation
                else
                    hmeas_OT=@(x)[x(1);x(2);x(3);x(4);x(5);x(6)]; % measurement equation (with velocity measurement)
                end
                [x_OT,P_OT]=ukf(fstate_OT,x_OT,P_OT,hmeas_OT,z_OT,Q_OT,R_OT);
            end
            
            latestOTData = data_OT{i};
            
        end
        % compute timestep to predict until the end of the time interval
        currentTimestep = (t - data_OT{RawDataOT_ind}.DeviceTimeStamp);        
        
    else % = no measurements arrived inbetween, compute timestep for prediction
        OnlyPredictionOT = true;
        currentTimestep = timestep_in_s;
        disp(['Only prediction at time: ' num2str(t) 's. (Optical kalman index number: ' num2str(KDataOT_ind) ')'])
    end
    
    % build A
    A_OT = eye(statesize);
    for j = 1:3 %6
        A_OT(j,j+3) = currentTimestep;
    end
    % state prediction
    x_minus_OT = A_OT * x_OT;
    P_minus_OT = A_OT * P_OT * A_OT' + Q_OT;
    P_OT = P_minus_OT;
    x_OT = x_minus_OT;
    
    %put filtered data into KalmanData struct
    KalmanDataOT{KDataOT_ind,1}.position = x_OT(1:3)';
    KalmanDataOT{KDataOT_ind,1}.speed = [x_dot_OT; y_dot_OT; z_dot_OT];
    KalmanDataOT{KDataOT_ind,1}.P = P_OT;
    KalmanDataOT{KDataOT_ind,1}.KalmanTimeStamp = t;
    KalmanDataOT{KDataOT_ind,1}.OnlyPrediction = OnlyPredictionOT;
    
    KDataOT_ind = KDataOT_ind + 1;
    
    
    % velocity update for EMT
    if (KDataEM_ind > 2)
        x_dot_EM = (KalmanDataEM{KDataEM_ind-1}.position(1) - KalmanDataEM{KDataEM_ind-2}.position(1)) / timestep_in_s;
        y_dot_EM = (KalmanDataEM{KDataEM_ind-1}.position(2) - KalmanDataEM{KDataEM_ind-2}.position(2)) / timestep_in_s;
        z_dot_EM = (KalmanDataEM{KDataEM_ind-1}.position(3) - KalmanDataEM{KDataEM_ind-2}.position(3)) / timestep_in_s;
    end
    
    % EM
    if(oldRawDataEM_ind~=RawDataEM_ind) % if there were new measurements since the last update
        OnlyPredictionEM = false;
        for i = oldRawDataEM_ind+1:RawDataEM_ind
            R = R_EM;
            % update velocity
            if strcmp(velocityUpdateScheme, 'LatestMeasuredData')
                diffTime = data_EMT{i}.DeviceTimeStamp - latestEMData.DeviceTimeStamp;
                if    diffTime < 0.1 ...% maximal one reading was dropped inbetween (assuming 20Hz rate)
                   && diffTime > 1000*eps 
                    
                    x_dot_EM = (data_EMT{i}.position(1) - latestEMData.position(1)) / diffTime;
                    y_dot_EM = (data_EMT{i}.position(2) - latestEMData.position(2)) / diffTime;
                    z_dot_EM = (data_EMT{i}.position(3) - latestEMData.position(3)) / diffTime;
                else
                    warning(['Kalman Position Data (EM) is taken to calculate velocity at time: ' num2str(data_EMT{i}.DeviceTimeStamp) 's. (DataEM index number: ' num2str(i) ')'])
                end
                z_EM = [ data_EMT{i}.position(1); data_EMT{i}.position(2); data_EMT{i}.position(3); x_dot_EM; y_dot_EM; z_dot_EM]; %measurement      
            end
            
            if strcmp(velocityUpdateScheme, 'Inherent')
                z_EM = [ data_EMT{i}.position(1); data_EMT{i}.position(2); data_EMT{i}.position(3)];
            end
            
            if(i==(oldRawDataEM_ind+1))
                currentTimestep = (data_EMT{i}.DeviceTimeStamp - (t-timestep_in_s));
            else
                currentTimestep = (data_EMT{i}.DeviceTimeStamp - data_EMT{i-1}.DeviceTimeStamp);
            end
            % build A
            A_EM = eye(statesize);
            for j = 1:3 %6 
                A_EM(j,j+3) = currentTimestep;
            end
                   
            %% Kalman algorithm (KF, EKF, UKF, CKF, ...)
                       
            if (KF==1)
                %KF
                % state prediction
                x_minus_EM = A_EM * x_EM;
                P_minus_EM = A_EM * P_EM * A_EM' + Q_EM; 
                % state update (correction by measurement)
                K_EM = (P_minus_EM * H_EM') / (H_EM * P_minus_EM * H_EM' + R_EM); %Kalman gain
                x_EM = x_minus_EM + K_EM * (z_EM - (H_EM * x_minus_EM));
                P_EM = (eye(statesize) - K_EM * H_EM ) * P_minus_EM;
            else
                %UKF
                fstate_EM=@(x)(x + [currentTimestep * x(4); currentTimestep * x(5); currentTimestep * x(6); zeros(statesize-3,1)]);  % nonlinear state equations
                if strcmp(velocityUpdateScheme, 'Inherent')
                    hmeas_EM=@(x)[x(1);x(2);x(3)]; % measurement equation
                else
                    hmeas_EM=@(x)[x(1);x(2);x(3);x(4);x(5);x(6)]; % measurement equation (with velocity measurement)
                end
                [x_EM,P_EM]=ukf(fstate_EM,x_EM,P_EM,hmeas_EM,z_EM,Q_EM,R_EM);
            end
            
            latestEMData = data_EMT{i};
            
        end
        % compute timestep to predict until the end of the time interval
        currentTimestep = (t - data_EMT{RawDataEM_ind}.DeviceTimeStamp);        
        
    else % = no measurements arrived inbetween, compute timestep for prediction
        OnlyPredictionEM = true;
        currentTimestep = timestep_in_s;
        disp(['Only prediction at time: ' num2str(t) 's. (EM kalman index number: ' num2str(KDataEM_ind) ')'])
    end
    
    % build A
    A_EM = eye(statesize);
    for j = 1:3 %6
        A_EM(j,j+3) = currentTimestep;
    end
    % state prediction
    x_minus_EM = A_EM * x_EM;
    P_minus_EM = A_EM * P_EM * A_EM' + Q_EM;
    P_EM = P_minus_EM;
    x_EM = x_minus_EM;
    
    %put filtered data into KalmanData struct
    KalmanDataEM{KDataEM_ind,1}.position = x_EM(1:3)';
    KalmanDataEM{KDataEM_ind,1}.speed = [x_dot_EM; y_dot_EM; z_dot_EM];
    KalmanDataEM{KDataEM_ind,1}.P = P_EM;
    KalmanDataEM{KDataEM_ind,1}.KalmanTimeStamp = t;
    KalmanDataEM{KDataEM_ind,1}.OnlyPrediction = OnlyPredictionEM;
    
    KDataEM_ind = KDataEM_ind + 1;
    
    % Update synchronous Kalman time
    t = t + timestep_in_s;
end


%% plots
% OT
numKalmanPts = size(KalmanDataOT,1);
for i = 1:numKalmanPts
    KalmanDataOT{i}.orientation = [.5 .5 .5 .5];
end

KalmanData_structarray = [KalmanDataOT{:}];

% EM
numKalmanPts = size(KalmanDataEM,1);
for i = 1:numKalmanPts
    KalmanDataEM{i}.orientation = [.5 .5 .5 .5];
end

KalmanDataOT_structarray = [KalmanDataOT{:}];
KalmanDataEM_structarray = [KalmanDataEM{:}];

% plot path in 3D
OT_points_cell = trackingdata_to_matrices(data_OT, 'cpp');
EMT_points_cell = trackingdata_to_matrices(data_EMT, 'cpp');

H_KalmanDataOT_cell = trackingdata_to_matrices(KalmanDataOT, 'cpp');
datafig = Plot_points(H_KalmanDataOT_cell, [], 3, 'o');
H_KalmanDataEM_cell = trackingdata_to_matrices(KalmanDataEM, 'cpp');
Plot_points(H_KalmanDataEM_cell, datafig, 2, 'o');

Plot_points(OT_points_cell, datafig, 3, 'x');
Plot_points(EMT_points_cell,datafig,2,'+');

KalmanPredictions = [KalmanData_structarray.OnlyPrediction];
predictionInds = find(KalmanPredictions);
if ~isempty(predictionInds)
    [x,y,z] = sphere(20);
    x = 2*x; % 2mm radius
    y = 2*y;
    z = 2*z;
    for i = predictionInds
        hold on
        surf(x+KalmanData{i}.position(1), y+KalmanData{i}.position(2), z+KalmanData{i}.position(3), 'edgecolor', 'none', 'facecolor', 'red', 'facealpha', 0.3)
        hold off
    end
end
%Plot_points(orig_cell,[], 3, 'x');
title('data EM: x, data OT: +, data filtered: o');

% plot velocities
VelocityFigure = figure;
title('Speeds [mm/s] in x, y, z direction over time in [s].')
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
KalmanCovariance = reshape(KalmanCovariance,statesize,statesize,numKalmanPts);
posvar = zeros(1,numKalmanPts);
speedvar = posvar;
for i = 1:numKalmanPts
posvar(i) = norm([KalmanCovariance(1,1,i) KalmanCovariance(2,2,i) KalmanCovariance(2,2,i)]);
speedvar(i) = norm([KalmanCovariance(4,4,i) KalmanCovariance(5,5,i) KalmanCovariance(6,6,i)]);
end
subplot(2,1,1)
plot(KalmanTime, sqrt(posvar), 'b--', KalmanTime, -sqrt(posvar), 'b--',...
    KalmanTime, repmat(sqrt(position_variance_OT),1,numKalmanPts), 'g', KalmanTime, repmat(-sqrt(position_variance_OT),1,numKalmanPts), 'g',...
    KalmanTime, repmat(sqrt(position_variance_EM),1,numKalmanPts), 'y', KalmanTime, repmat(-sqrt(position_variance_EM),1,numKalmanPts), 'y')
title('position sdev in blue--, pos noise sdev of Optical in green, of EM in yellow')
subplot(2,1,2)
plot(KalmanTime, sqrt(speedvar),'r--', KalmanTime, -sqrt(speedvar), 'r--')
title('speed sdev')

clear KalmanData_structarray
end



















