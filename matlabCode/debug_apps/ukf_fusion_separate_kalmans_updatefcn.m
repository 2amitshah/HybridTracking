% This is a debug implementation of the Unscented Kalman Filter.
% There is deterministic sampling of sigma points and then a transform
% to compute mean and covariance of the forecasted values.
% This is for statistical linear motion.
% This function is for combining the data of the two sensors. This means that
% the data of OT and EM is put into a Kalman filter as soon as it is
% available. Depending on the system from which the data is coming, the
% R-matrix (Measurement noise) is set accordingly.

function [KalmanDataOT, KalmanDataEM] = ukf_fusion_separate_kalmans_updatefcn(filenames_struct, kalmanfrequencyHz, verbosity)
%% read arguments, set defaults
KF = 0; % 0: use UKF algorithm, 1: use simple Kalman algorithm

EMCSspace = 0; % 1: map everything into EMCS coordinate system, 0: map everything into OCS coordinate system.

% 'Inherent': do not create a virtual velocity measurement,
% 'LatestMeasuredData': create a velocity measurement by differencing the two latest measured data points of one modality
% 'LatestKalmanData': create a velocity measurement by differencing the two
% latest filtered points of the Kalman Output.
velocityUpdateScheme = 'LatestMeasuredData';

estimateOrientation = 1; % 0: do not estimae Orientation, only Position, 1: also estimate Orientation (nonlinearly)

% 'Inherent': do not create a virtual angular velocity measurement,
% 'LatestMeasuredData': create a angular velocity measurement by differencing the two latest measured data points of one modality
% 'LatestKalmanData': create a angular velocity measurement by differencing the two
% latest filtered points of the Kalman Output.
angvelUpdateScheme = 'LatestMeasuredData';


if exist('filenames_struct', 'var') && isstruct(filenames_struct)
    testrow_name_EM = filenames_struct.EMfiles;
    testrow_name_OT = filenames_struct.OTfiles;
    path = filenames_struct.folder;
else
    warning('GeneralWarning:pathStruct',['Please use the new filenames_struct-feature.\n'...
        ' ''path'' can now be a struct, so you don''t always have to change the default ''testrow_name_EMT'' and ''testrow_name_OT''.'])
end

if ~exist('verbosity', 'var')
    verbosity = 'vRelease';
end
if ~exist('kalmanfrequencyHz','var')
    kalmanfrequencyHz = 17;
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
% data_EM_Sensor1 = data_EMT_tmp(1:size(data_EMT_tmp,1),1);

%% perform synchronization
EM_minus_OT_offset = sync_from_file(filenames_struct, 'vRelease', 'device');
numPtsEMT = size(data_EMT_tmp,1);
numEMs = size(data_EMT_tmp,2);
for i = 1:numPtsEMT
    for j = 1:numEMs
        if ~isempty(data_EMT_tmp{i,j})
            % move EM timestamps into timeframe of Optical (because Optical is our common reference)
            data_EMT_tmp{i,j}.DeviceTimeStamp = data_EMT_tmp{i,j}.DeviceTimeStamp - EM_minus_OT_offset;
        end
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
interval = obtain_boundaries_for_interpolation(data_OT_tmp, data_EMT_tmp, 'device');
startTime = interval(1);
% startTime = data_OT_tmp{3}.DeviceTimeStamp;
endTime = interval(2);

%% get Y, ( = H_OCS_to_EMCS)
load('H_OT_to_EMT');
[Y,YError] = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease','device');

%% compute homogenuous matrices from struct data
[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT_tmp, 'CppCodeQuat');
[H_OT_to_OCS_cell] = trackingdata_to_matrices(data_OT_tmp, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS_cell{1};
H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
if numEMs > 1
    H_EMT2_to_EMCS = H_EMT_to_EMCS_cell{2};
%     load('H_OT_to_EMT2.mat'); % TODO make a OT to EMT2 calibration as
%     well
    load('H_EMT2_to_EMT1');
    H_OT_to_EMT2 = H_EMT2_to_EMT1 \ H_OT_to_EMT;
    if numEMs > 2
    H_EMT3_to_EMCS = H_EMT_to_EMCS_cell{3};
%     load('H_OT_to_EMT3.mat');
    load('H_EMT3_to_EMT1');
    H_OT_to_EMT3 = H_EMT3_to_EMT1 \ H_OT_to_EMT; % TODO make a OT to EMT3 calibration as
%     well
    end
end


%% calculate OT position in EMCS frame

% OT
numPtsOT = size(data_OT_tmp,1);
H_OT_to_EMCS = zeros(4,4,numPtsOT);
data_OT_to_EMCS = cell(numPtsOT,1);

for i = 1:numPtsOT
    H_OT_to_EMCS(:,:,i) = Y*H_OT_to_OCS(:,:,i);
    if(~isempty(data_OT_tmp{i}) && data_OT_tmp{i}.valid == 1)
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
data_OT_to_EMCS_by_EMT = cell(numPtsEMT,numEMs);

for j = 1:numEMs
    if j == 2
        H_OT_to_EMT = H_OT_to_EMT2;
        H_EMT_to_EMCS = H_EMT2_to_EMCS;
    end
    if j == 3
        H_OT_to_EMT = H_OT_to_EMT3;
        H_EMT_to_EMCS = H_EMT3_to_EMCS;
    end
    for i = 1:numPtsEMT
        H_OT_to_EMCS_by_EMT(:,:,i) = H_EMT_to_EMCS(:,:,i) * H_OT_to_EMT;
        if(~isempty(data_EMT_tmp{i,j}) && data_EMT_tmp{i,j}.valid == 1)
            data_OT_to_EMCS_by_EMT{i,j}.DeviceTimeStamp = data_EMT_tmp{i,j}.DeviceTimeStamp;      
            data_OT_to_EMCS_by_EMT{i,j}.position = (H_OT_to_EMCS_by_EMT(1:3,4,i))';
            data_OT_to_EMCS_by_EMT{i,j}.orientation = [(rot2quat(H_OT_to_EMCS_by_EMT(1:3, 1:3, i)))' 1];   
            data_OT_to_EMCS_by_EMT{i,j}.valid = data_EMT_tmp{i,j}.valid;
        else
            data_OT_to_EMCS_by_EMT{i,j}.valid = 0;
        end
    end
end

%% calculate OT position in OCS frame
% OT
data_OT_to_OCS = data_OT_tmp;

% EMT
H_OT_to_OCS_by_EMT = zeros(4,4,numPtsEMT);
data_OT_to_OCS_by_EMT = cell(numPtsEMT,numEMs);
for j = 1:numEMs
    if j == 2
        H_OT_to_EMT = H_OT_to_EMT2;
        H_EMT_to_EMCS = H_EMT2_to_EMCS;
    end
    if j == 3
        H_OT_to_EMT = H_OT_to_EMT3;
        H_EMT_to_EMCS = H_EMT3_to_EMCS;
    end
    for i = 1:numPtsEMT
        H_OT_to_OCS_by_EMT(:,:,i) = Y \ H_EMT_to_EMCS(:,:,i) * H_OT_to_EMT;
        if(~isempty(data_EMT_tmp{i,j}) && data_EMT_tmp{i,j}.valid == 1)
            data_OT_to_OCS_by_EMT{i,j}.DeviceTimeStamp = data_EMT_tmp{i,j}.DeviceTimeStamp;      
            data_OT_to_OCS_by_EMT{i,j}.position = (H_OT_to_OCS_by_EMT(1:3,4,i))';
            data_OT_to_OCS_by_EMT{i,j}.orientation = [(rot2quat(H_OT_to_OCS_by_EMT(1:3, 1:3, i)))' 1];   
            data_OT_to_OCS_by_EMT{i,j}.valid = data_EMT_tmp{i,j}.valid;
        else
            data_OT_to_OCS_by_EMT{i,j}.valid = 0;
        end
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

% convert quaternion to explicit XYZ-Euler
RotTwoLatest = quat2dcm([data_OT{3}.orientation(4) data_OT{3}.orientation(1) data_OT{3}.orientation(2) data_OT{3}.orientation(3) ;...
                        [data_OT{2}.orientation(4) data_OT{2}.orientation(1) data_OT{2}.orientation(2) data_OT{2}.orientation(3) ]]);
RotDiff = RotTwoLatest(:,:,1) \ RotTwoLatest(:,:,2); % goes from latest to recent, should be in the right direction
[x_angle, y_angle, z_angle]= dcm2angle(RotDiff, 'XYZ');
x_angvel = (x_angle)/timestep23;
y_angvel = (y_angle)/timestep23;
z_angvel = (z_angle)/timestep23;

% state vector without acceleration but with attitude (q) and angular
% velocity
initx = [
        data_OT{3}.position(1);
        data_OT{3}.position(2);
        data_OT{3}.position(3);
        x_dot;
        y_dot;
        z_dot;
        data_OT{3}.orientation(4);
        data_OT{3}.orientation(1);
        data_OT{3}.orientation(2);
        data_OT{3}.orientation(3);
        x_angvel;
        y_angvel;
        z_angvel
        ];

x = initx;

x_EM = cell(1, numEMs);
for j = 1:numEMs
x_EM{j} = x;
end
x_OT = x;

statesize = numel(x);
H = zeros(statesize,statesize);
if(strcmp(velocityUpdateScheme, 'Inherent'))
    if estimateOrientation == 0
        observationsize = 3;        % position is observed
        H(1:3,1:3) = eye(3);
    elseif estimateOrientation == 1
        if (strcmp(angvelUpdateScheme, 'Inherent'))
            observationsize = 7;    % position and quaternion are observed
            H(1:3,1:3) = eye(3); H(7:10,7:10) = eye(4);
        else
            observationsize = 10;   % position, quaternion and angvel are observed
            H(1:3,1:3) = eye(3); H(7:13,7:13) = eye(7);
        end
    end
else
    if estimateOrientation == 0
        observationsize = 6;        % position and velocity are observed
        H(1:6,1:6) = eye(6);
    elseif estimateOrientation == 1
        if (strcmp(angvelUpdateScheme, 'Inherent'))
            observationsize = 10;   % position, velocity and quaternion are observed
            H(1:10,1:10) = eye(10);
        else
            observationsize = 13;   % everything is observed
            H(1:13,1:13) = eye(13);
        end
    end
end

% state transition matrix A
A = eye(statesize);
for i = 1:3 %6
    A(i,i+3) = timestep_in_s;
end
disp('A for constant velocity case')
disp(A)
% A_EM = A;
% A_OT = A;

% H = zeros(observationsize,statesize);
% H(1:observationsize, 1:observationsize) = eye(observationsize);
disp('H for constant velocity case')
disp(H)
H_EM = H;
H_OT = H;

%initial state covariance (P)
initP = .05 * eye(statesize);
initP(4:6,4:6) = 0.1 * eye(3);
% initP(7:9,7:9) = 1400 * eye(3);
P = initP;

P_EM = cell(1, numEMs);
for j = 1:numEMs
P_EM{j} = P;
end

P_OT = P;

% process noise covariance matrix Q
Q = 0.5 * eye(statesize);
% if ~(strcmp(velocityUpdateScheme, 'Inherent'))
    Q(4:6,4:6) = 0.5 * kalmanfrequencyHz * 2 * eye(3); % this can't be quite right
% end
% Q(7:9,7:9) = 100 * eye(3);
Q_EM = Q;
Q_OT = Q;

% measurement noise covariance matrix R
XError = 1; % error remaining from the calibration

% this error should exclude static errors such as from Calibration and Y
% matrix estimation. It should only depict how reliable EM and OT are in
% their respective coordinate frames. The MASTER Kalman (foundation kalman
% filter) then has to know how good they are aligned to the current global
% coordinate system. The individual fitlers do not take care of that.
if EMCSspace == 1
%     position_variance_OT = (0.25 + YError)^2;
%     position_variance_EM = (1 + XError)^2;
else
%     position_variance_OT = (0.25)^2;
%     position_variance_EM = (1 + XError + YError)^2;
end

position_variance_OT = (0.25)^2; % NDI Polaris product description
position_variance_EM = (0.9)^2; % Maier-Hein's paper 2011

R_OT = position_variance_OT*eye(sum(diag(H))); %the higher the value, the less the measurement is trusted
if ~(strcmp(velocityUpdateScheme, 'Inherent'))
    R_OT(4:6,4:6) =  2 * position_variance_OT * kalmanfrequencyHz * eye(3);
end
% if estimateOrientation == 1
%     
% end
R_EM = position_variance_EM*eye(sum(diag(H)));
if ~(strcmp(velocityUpdateScheme, 'Inherent'))
    R_EM(4:6,4:6) =  2 * position_variance_EM * kalmanfrequencyHz * eye(3);
end

%% perform Kalman filtering, take whatever is available (OT or EM) and feed it into the Kalman filter
% start Filter

latestOTData = data_OT{4};
RawDataOT_ind = 0;
KDataOT_ind = 1;

numKalmanPts = numel(startTime:timestep_in_s:endTime);
KalmanDataOT = cell(numKalmanPts, 1);
KalmanDataEM = cell(numKalmanPts, numEMs);
latestEMData = cell(1,numEMs);
RawDataEM_ind = cell(1,numEMs);
KDataEM_ind = cell(1,numEMs);
i = 1;
for j = 1:numEMs
    while(isempty(data_EMT{i,j}))
        i = i+1;
    end
    latestEMData{j} = data_EMT{i,j};
    i = 1;
    RawDataEM_ind{j} = 0;
    KDataEM_ind{j} = 1;
end

KalmanDataMaster = KalmanDataOT;
KMasterInd = 1;

t = startTime;

while(t <= endTime + 1000*eps)

    [KalmanDataOT, latestOTData, RawDataOT_ind, KDataOT_ind] = KalmanUpdate(t, RawDataOT_ind, data_OT, latestOTData,...
        x_OT, P_OT, Q_OT, H_OT, R_OT, statesize, timestep_in_s, KDataOT_ind, KalmanDataOT,...
        estimateOrientation, velocityUpdateScheme, angvelUpdateScheme, KF);
    for j = 1:numEMs
        [KalmanDataEM(:,j), latestEMData{j}, RawDataEM_ind{j}, KDataEM_ind{j}] = KalmanUpdate(t, RawDataEM_ind{j}, data_EMT(:,j), latestEMData{j},...
            x_EM{j}, P_EM{j}, Q_EM, H_EM, R_EM, statesize, timestep_in_s, KDataEM_ind{j}, KalmanDataEM(:,j),...
            estimateOrientation, velocityUpdateScheme, angvelUpdateScheme, KF);
    end

%%%%%%%%%%%%%%%%
% master filter
%%%%%%%%%%%%%%%%

    OptPartX = KalmanDataOT{KDataOT_ind-1}.P \ KalmanDataOT{KDataOT_ind-1}.x;
    OptPartPinverted = KalmanDataOT{KDataOT_ind-1}.P \ eye(statesize);
    Xm = OptPartX;
    Pminverted = OptPartPinverted;
    for j = 1:numEMs
        Pminverted = Pminverted + KalmanDataEM{ KDataEM_ind{j}-1, j}.P \ eye(statesize);
        Xm = Xm + KalmanDataEM{KDataEM_ind{j}-1, j}.P \ KalmanDataEM{KDataEM_ind{j}-1,j}.x;
    end

    KalmanDataMaster{KMasterInd}.P = Pminverted \ eye(statesize);
    KalmanDataMaster{KMasterInd}.x = KalmanDataMaster{KMasterInd}.P * Xm;

    KMasterInd = KMasterInd + 1;

    % Update synchronous Kalman time
    t = t + timestep_in_s;
end


%% plots
if strcmp(verbosity, 'vRelease')
    close all;
end
numKalmanPtsOT = size(KalmanDataOT,1);
numKalmanPtsEM = size(KalmanDataEM,1);
if estimateOrientation == 0
% OT
for i = 1:numKalmanPtsOT
    KalmanDataOT{i}.orientation = [.5 .5 .5 .5];
end
% EM
for i = 1:numKalmanPtsEM
    KalmanDataEM{i}.orientation = [.5 .5 .5 .5];
end
end

KalmanDataOT_structarray = [KalmanDataOT{:}];
KalmanDataEM_structarray = [KalmanDataEM{:}];
for j = 1:numEMs
KalmanDataEM_structarraycell{j} = [KalmanDataEM{:,j}];
end

% plot path in 3D
OT_points_cell = trackingdata_to_matrices(data_OT, 'cpp');
EMT_points_cell = trackingdata_to_matrices(data_EMT, 'cpp');

H_KalmanDataOT_cell = trackingdata_to_matrices(KalmanDataOT, 'ndi');
datafig = Plot_points(H_KalmanDataOT_cell, [], 4, 'o');
H_KalmanDataEM_cell = trackingdata_to_matrices(KalmanDataEM, 'ndi');
Plot_points(H_KalmanDataEM_cell, datafig, 1, 'o');

Plot_points(OT_points_cell, datafig, 4, 'x');
Plot_points(EMT_points_cell,datafig, 1, '+');

KalmanPredictionsOT = [KalmanDataOT_structarray.OnlyPrediction];
predictionIndsOT = find(KalmanPredictionsOT);

if ~isempty(predictionIndsOT)
    [x,y,z] = sphere(20);
    x = 2*x; % 2mm radius
    y = 2*y;
    z = 2*z;
    for i = predictionIndsOT
        hold on
        surf(x+KalmanDataOT{i}.position(1), y+KalmanDataOT{i}.position(2), z+KalmanDataOT{i}.position(3), 'edgecolor', 'none', 'facecolor', 'red', 'facealpha', 0.3)
        hold off
    end
end

KalmanPredictionsEM = [KalmanDataEM_structarray.OnlyPrediction];
predictionIndsEM = find(KalmanPredictionsEM);

if ~isempty(predictionIndsEM)
    for i = predictionIndsEM
        hold on
        surf(x+KalmanDataEM{i}.position(1), y+KalmanDataEM{i}.position(2), z+KalmanDataEM{i}.position(3), 'edgecolor', 'none', 'facecolor', 'green', 'facealpha', 0.3)
        hold off
    end
end
drawnow
% Plot orientations
% OrientaionFigure = figure;
Plot_frames_gentle(OT_points_cell, datafig, 'r');
Plot_frames_gentle(EMT_points_cell, datafig, 'g');
Plot_frames_gentle(H_KalmanDataOT_cell, datafig, 'r--1');
Plot_frames_gentle(H_KalmanDataEM_cell, datafig, 'g--1');

%Plot_points(orig_cell,[], 3, 'x');
title('data OT: x, data EM: +, data filtered: o');

KalmanTimeOT = [KalmanDataOT_structarray.KalmanTimeStamp];
KalmanTimeEM = [KalmanDataEM_structarray.KalmanTimeStamp];
KalmanTimeEM = KalmanTimeEM(1:numKalmanPtsEM);
if strcmp(verbosity, 'vDebug')
    % Plot velocities
    VelocityFigure = figure;
    title('Speeds [mm/s] in x, y, z direction over time in [s].')
    KalmanSpeedsOT = [KalmanDataOT_structarray.speed];
    
    subplot(3,2,1)
    plot(KalmanTimeOT, KalmanSpeedsOT(1,:), 'r')
    title('x\_dot of Optical')
    subplot(3,2,3)
    plot(KalmanTimeOT, KalmanSpeedsOT(2,:), 'g')
    title('y\_dot')
    subplot(3,2,5)
    plot(KalmanTimeOT, KalmanSpeedsOT(3,:), 'b')
    title('z\_dot')

    KalmanSpeedsEM = [KalmanDataEM_structarray.speed];
    
    subplot(3,2,2)
    plot(KalmanTimeEM, KalmanSpeedsEM(1,:), 'r')
    title('x\_dot of Electromagnetic')
    subplot(3,2,4)
    plot(KalmanTimeEM, KalmanSpeedsEM(2,:), 'g')
    title('y\_dot')
    subplot(3,2,6)
    plot(KalmanTimeEM, KalmanSpeedsEM(3,:), 'b')
    title('z\_dot')
end

% plot development of P entries
CovarianceFigure = figure;
title('Diagonal elements of state covariance P.')

KalmanCovarianceOT = [KalmanDataOT_structarray.P];
KalmanCovarianceOT = reshape(KalmanCovarianceOT,statesize,statesize,numKalmanPtsOT);
posvarOT = zeros(1,numKalmanPtsOT);
speedvarOT = posvarOT;
quaternionvarOT = posvarOT;
angvelvarOT = posvarOT;

KalmanCovarianceEM = cell(1,numEMs);
for j = 1:numEMs
    KalmanCovarianceEM{j} = [KalmanDataEM_structarraycell{j}.P];
    KalmanCovarianceEM{j} = reshape(KalmanCovarianceEM{j},statesize,statesize,numKalmanPtsEM);
end
posvarEM = zeros(numKalmanPtsEM, numEMs);
speedvarEM = posvarEM;
quaternionvarEM = posvarEM;
angvelvarEM = posvarEM;

for i = 1:numKalmanPtsOT
posvarOT(i) = norm(diag(KalmanCovarianceOT(1:3,1:3,i)));
speedvarOT(i) = norm(diag(KalmanCovarianceOT(4:6,4:6,i)));
quaternionvarOT(i) = norm(diag(KalmanCovarianceOT(7:10,7:10,i)));
angvelvarOT(i) = norm(diag(KalmanCovarianceOT(11:13,11:13,i)));
end

for j = 1:numEMs
    for i = 1:numKalmanPtsEM
    posvarEM(i,j) = norm(diag(KalmanCovarianceEM{j}(1:3,1:3,i)));
    speedvarEM(i,j) = norm(diag(KalmanCovarianceEM{j}(4:6,4:6,i)));
    quaternionvarEM(i,j) = norm(diag(KalmanCovarianceEM{j}(7:10,7:10,i)));
    angvelvarEM(i,j) = norm(diag(KalmanCovarianceEM{j}(11:13,11:13,i)));
    end
end
subplot(2,numEMs+1,1)
plot(KalmanTimeOT, sqrt(posvarOT), 'r--', KalmanTimeOT, -sqrt(posvarOT), 'r--',...
    KalmanTimeOT, repmat(sqrt(position_variance_OT),1,numKalmanPtsOT), 'r', KalmanTimeOT, repmat(-sqrt(position_variance_OT),1,numKalmanPtsOT), 'r',...
    KalmanTimeOT, repmat(sqrt(position_variance_EM),1,numKalmanPtsOT), 'g', KalmanTimeOT, repmat(-sqrt(position_variance_EM),1,numKalmanPtsOT), 'g')
title('Optical: position sdev in red--, pos noise sdev of Optical in red, of EM in green')
subplot(2,numEMs+1,numEMs+2)
plot(KalmanTimeOT, sqrt(speedvarOT),'r--', KalmanTimeOT, -sqrt(speedvarOT), 'r--')
title('speed sdev')
for j = 1:numEMs
    subplot(2,numEMs+1,j+1)
    plot(KalmanTimeEM, sqrt(posvarEM(:,j)), 'g--', KalmanTimeEM, -sqrt(posvarEM(:,j)), 'g--',...
        KalmanTimeEM, repmat(sqrt(position_variance_OT),1,numKalmanPtsEM), 'r', KalmanTimeEM, repmat(-sqrt(position_variance_OT),1,numKalmanPtsEM), 'r',...
        KalmanTimeEM, repmat(sqrt(position_variance_EM),1,numKalmanPtsEM), 'g', KalmanTimeEM, repmat(-sqrt(position_variance_EM),1,numKalmanPtsEM), 'g')
    title('Electromagnetic: position sdev in green--, pos noise sdev of Optical in red, of EM in green')
    subplot(2,numEMs+1,j+numEMs+2)
    plot(KalmanTimeEM, sqrt(speedvarEM(:,j)),'g--', KalmanTimeEM, -sqrt(speedvarEM(:,j)), 'g--')
    title('speed sdev')
end

if estimateOrientation == 1
    subplot(2,numEMs+1,1)
    hold on
    plot(KalmanTimeOT, sqrt(quaternionvarOT), 'r--', KalmanTimeOT, -sqrt(quaternionvarOT), 'r--')
    hold off
    title('Optical: position sdev in red--, pos noise sdev of Optical in red, of EM in green')
    subplot(2,numEMs+1,numEMs+2)
    hold on
    plot(KalmanTimeOT, sqrt(angvelvarOT),'r--', KalmanTimeOT, -sqrt(angvelvarOT), 'r--')
    hold off
    title('speed sdev')
for j = 1:numEMs
    subplot(2,numEMs+1,j+1)
    hold on
    plot(KalmanTimeEM, sqrt(quaternionvarEM(:,j)), 'g--', KalmanTimeEM, -sqrt(quaternionvarEM(:,j)), 'g--')
    hold off
    title('Electromagnetic: position sdev in green--, pos noise sdev of Optical in red, of EM in green')
    subplot(2,numEMs+1,j+numEMs+2)
    hold on
    plot(KalmanTimeEM, sqrt(angvelvarEM(:,j)),'g--', KalmanTimeEM, -sqrt(angvelvarEM(:,j)), 'g--')
    hold off
    title('speed sdev')
end
end

% Plot of the Residual of Kalman Prediction and respective Measurement
DeviationFigure = figure;
title('Residual of Kalman Prediction and respective Measurement (input for IMM algorithm)')

% OT
KalmanDeviationOT = [KalmanDataOT_structarray.Deviation];

posdevOT = zeros(1,size(KalmanDeviationOT,2));
speeddevOT = posdevOT;
quaterniondevOT = posdevOT;
angveldevOT = posdevOT;

for i = 1:size(KalmanDeviationOT,2)
        posdevOT(i) = norm(KalmanDeviationOT(1:3,i));
    if~(strcmp(velocityUpdateScheme, 'Inherent'))
        speeddevOT(i) = norm(KalmanDeviationOT(4:6,i));
    end
    if estimateOrientation == 1
        quaterniondevOT(i) = norm(KalmanDeviationOT(7:10,i));
        if~(strcmp(angvelUpdateScheme, 'Inherent'))
            angveldevOT(i) = norm(KalmanDeviationOT(11:13,i));
        end
    end
end

subplot(2,numEMs+1,1)
plot(KalmanTimeOT(~KalmanPredictionsOT), posdevOT, 'r')
title('Optical: Residual of position')
if estimateOrientation == 1
    hold on
    plot(KalmanTimeOT(~KalmanPredictionsOT), quaterniondevOT, 'r-*')
    hold off
end
    
subplot(2,numEMs+1,numEMs+2)
if~(strcmp(velocityUpdateScheme, 'Inherent'))
    plot(KalmanTimeOT(~KalmanPredictionsOT), speeddevOT, 'r')
    title('Optical: Residual of velocity')
end
if estimateOrientation == 1
    if~(strcmp(angvelUpdateScheme, 'Inherent'))
        hold on
        plot(KalmanTimeOT(~KalmanPredictionsOT), angveldevOT, 'r-*')
        hold off
    end
end

% EM

posdevEM = zeros(numKalmanPtsEM, numEMs);
speeddevEM = posdevEM;
quaterniondevEM = posdevEM;
angveldevEM = posdevEM;
KalmanDeviationEM = cell(1, numEMs);
KalmanPredictionsEM = KalmanDeviationEM;
for j=1:numEMs
    KalmanDeviationEM{j} = [KalmanDataEM_structarraycell{j}.Deviation];
    KalmanPredictionsEM{j} = [KalmanDataEM_structarraycell{j}.OnlyPrediction];
    
    for i = 1:size(KalmanDeviationEM{j},2)
            posdevEM(i,j) = norm(KalmanDeviationEM{j}(1:3,i));
        if~(strcmp(velocityUpdateScheme, 'Inherent'))
            speeddevEM(i,j) = norm(KalmanDeviationEM{j}(4:6,i));
        end
        if estimateOrientation == 1
            quaterniondevEM(i,j) = norm(KalmanDeviationEM{j}(7:10,i));
            if~(strcmp(angvelUpdateScheme, 'Inherent'))
                angveldevEM(i,j) = norm(KalmanDeviationEM{j}(11:13,i));
            end
        end
    end
end


for j = 1:numEMs
    subplot(2,numEMs+1,j+1)
    plot(KalmanTimeEM(~KalmanPredictionsEM{j}), posdevEM(~KalmanPredictionsEM{j},j), 'g')
    title('Electromagnetic: Residual of position')
    if estimateOrientation == 1
        hold on
        plot(KalmanTimeEM(~KalmanPredictionsEM{j}), quaterniondevEM(~KalmanPredictionsEM{j},j), 'g-*')
        hold off
    end

    subplot(2,numEMs+1,j+numEMs+2)
    if~(strcmp(velocityUpdateScheme, 'Inherent'))
        plot(KalmanTimeEM(~KalmanPredictionsEM{j}), speeddevEM(~KalmanPredictionsEM{j},j), 'g')
    title('Electromagnetic: Residual of velocity')
    end
    if estimateOrientation == 1
        if~(strcmp(angvelUpdateScheme, 'Inherent'))
            hold on
            plot(KalmanTimeEM(~KalmanPredictionsEM{j}), angveldevEM(~KalmanPredictionsEM{j},j), 'g-*')
            hold off
        end
    end
end


clear KalmanDataOT_structarray KalmanDataEM_structarray
end



















