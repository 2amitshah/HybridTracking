% this function is for combining the data of the two sensors. Tthis means that
% we use the OT data (at the position of EM) when it is available because it is usually more
% accurate and the EM data (corrected by the known distortion field)
% otherwise. In order to overcome jittering and also small changes at the
% points where we change from OT to EM and back we use a Kalman filter for
% smoothing and correcting.

function datafiltered = common_OT_EM_kalman(path, testrow_name_EMT, testrow_name_OT, H_OT_to_EMT)

close all;
if ~exist('path','var')
     pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
     path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
 end
 if ~exist('H_OT_to_EMT','var')
     load(which('H_OT_to_EMT.mat'));
 end
 
 if ~exist('testrow_name_EMT','var')
    testrow_name_EMT = 'EMTrackingcont_1';
 end
 
 if ~exist('testrow_name_OT','var')
    testrow_name_OT = 'OpticalTrackingcont_1';
 end
 
 
% get Y, equal to EMCS_to_OCS
Y = polaris_to_aurora(path, H_OT_to_EMT,'cpp');
 
%% get the improved position of EM 1 and EM 2 (and EM 3, if available) at the position of EM 1
% (data_EM_common) and the data of OT (data_OT_common) at the same synthetic timestamps

[~, ~, data_EM_common, data_OT_common] =  OT_common_EMT_at_synthetic_timestamps_distortion_correction(path, testrow_name_EMT,testrow_name_OT);

%data_EM_synth = syntheticPositions();

%prepare data
numPts = size(data_EM_common,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

% Relevant matrix for computing transformations
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM_common, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT_common, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};


%% calculate where EM tracker should be
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
H_diff_EMT_to_EMCS = zeros(4,4,numPts);
translation_EMTcell = cell(1,2);

data_EM_common_by_OT = cell(numPts,1);

for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT; %data_EM_common_by_OT
    data_EM_common_by_OT{i}.valid = 0;
    if(data_OT_common{i}.valid == 1 && data_EM_common{i}.valid == 1)
        H_diff_EMT_to_EMCS(:,:,i) = inv(H_EMT_to_EMCS(:,:,i))*H_EMT_to_EMCS_by_OT(:,:,i);
        translation_EMTcell{1}.vector(:,i) = H_EMT_to_EMCS_by_OT(1:3,4,i);
        translation_EMTcell{2}.vector(:,i) = H_EMT_to_EMCS(1:3,4,i);
    
        data_EM_common_by_OT{i}.TimeStamp = data_OT_common{i}.TimeStamp;      
        data_EM_common_by_OT{i}.position = transpose(H_EMT_to_EMCS_by_OT(1:3,4,i)) ;
        data_EM_common_by_OT{i}.orientation = transpose(rot2quat_q41(H_EMT_to_EMCS_by_OT(1:3, 1:3, i)));   
        data_EM_common_by_OT{i}.valid = data_OT_common{i}.valid;
    end
end

%% combine the data of the two sensors
data = data_EM_common_by_OT;
for i = 1:numPts
    if (~(data{i}.valid) && data_EM_common{i}.valid)
        data{i} = data_EM_common{i};
    end
end


%% do Kalman filtering with data
timestep_in_s = (data{2}.TimeStamp - data{1}.TimeStamp) / 10^9;

% initialize matrices and vectors

% initial state vector init_x, start with timestep 3 (in order to have the first and second
% derivative)
x_dot = (data{3}.position(1) - data{2}.position(1))/timestep_in_s;
y_dot = (data{3}.position(2) - data{2}.position(2))/timestep_in_s;
z_dot = (data{3}.position(3) - data{2}.position(3))/timestep_in_s;
x_2dot = (x_dot - ((data{2}.position(1) - data{1}.position(1))/timestep_in_s)) / timestep_in_s;
y_2dot = (y_dot - ((data{2}.position(2) - data{1}.position(2))/timestep_in_s)) / timestep_in_s;
z_2dot = (z_dot - ((data{2}.position(3) - data{1}.position(3))/timestep_in_s)) / timestep_in_s;

initx = [data{3}.position(1); data{3}.position(2); data{3}.position(3); x_dot; y_dot; z_dot; x_2dot; y_2dot; z_2dot ] 

statesize = 9;
observationsize = 3;
% state transition matrix A
A = eye(statesize, statesize);
for i = 1:6
    A(i,i+3) = timestep_in_s;
end
for i = 1:3
    A(i,i+6) = .5 * timestep_in_s^2;
end
A

H = zeros(observationsize,statesize);
H(1:observationsize, 1:observationsize) = eye(observationsize,observationsize);
H

% estimate measurement noise covariance matrix R using some of the first measurements
amountForCov = 150;
firstVals = zeros(amountForCov,size(initx));
for i = 1:amountForCov
   firstVals(i,1) = data{i+2}.position(1);
   firstVals(i,2) = data{i+2}.position(2);
   firstVals(i,3) = data{i+2}.position(3);
   x_dot = (data{i+2}.position(1) - data{i+1}.position(1))/timestep_in_s;
   y_dot = (data{i+2}.position(2) - data{i+1}.position(2))/timestep_in_s;
   z_dot = (data{i+2}.position(3) - data{i+1}.position(3))/timestep_in_s;
   firstVals(i,4) = x_dot;
   firstVals(i,5) = y_dot;
   firstVals(i,6) = z_dot;
   x_2dot = (x_dot - ((data{i+1}.position(1) - data{i}.position(1))/timestep_in_s)) / timestep_in_s;
   y_2dot = (y_dot - ((data{i+1}.position(2) - data{i}.position(2))/timestep_in_s)) / timestep_in_s;
   z_2dot = (z_dot - ((data{i+1}.position(3) - data{i}.position(3))/timestep_in_s)) / timestep_in_s;
   firstVals(i,7) = x_2dot;
   firstVals(i,8) = y_2dot;
   firstVals(i,9) = z_2dot;   
end
firstVals
R_OT = cov(firstVals);
R_OT = R_OT(1:observationsize,1:observationsize);
%r=0.03;    %std of measurement 

r=sqrt(.01);    %std of measurement 
R_OT=r^2;%*eye(3);
%R_OT=.1*eye(3);

% estimate process noise covariance matrix Q
q=0.1;    %std of process 
Q=q^2*eye(statesize);

% initial state covariance (P)
initV = eye(statesize);

datafiltered = cell(numPts-3,1);

% observation at time t, coming from data
y = zeros(3,numPts);
for i = 1:numPts
    if(data{i}.valid)
        y(1:3,i) = data{i}.position;
    else
        y(1:3,i) = y(1:3,i-1)
    end
   %y(2,i)
   %y(3,i)
end

Q = 0.1*eye(statesize);
R = 1*eye(observationsize);

[xfilt, Vfilt, VVfilt, loglik] = kalman_filter(y, A, H, Q, R, initx, initV);

for i = 4:numPts
    datafiltered{i}.position = xfilt(1:observationsize,i-3);
    datafiltered{i}.speed = xfilt(4:6,i-3);
    datafiltered{i}.acceleration = xfilt(7:9,i-3);
end

close all;
figure(1)
for i = 4:numPts
    if data{i}.valid
        plot3(data{i}.position(1),data{i}.position(2),data{i}.position(3),'rx');
        hold on
        plot3(datafiltered{i}.position(1),datafiltered{i}.position(2),datafiltered{i}.position(3),'bx');
        hold on
    end
end
%plotEnvironment(1, H_OT_to_EMT, Y)
title('Position of common EMT1 sensor (red, filtered: blue) at synthetic timestamps')

figure(4)
for i = 4:size(datafiltered,1)
    plot(i,datafiltered{i}.speed(1),'r');
    hold on
    plot(i,datafiltered{i}.speed(2),'g');
    plot(i,datafiltered{i}.speed(3),'b');        
end
title('speed of filtered signal in x-, y- and z-direction (r,g,b)');
figure(5)
for i = 4:size(datafiltered,1)
    plot(i,datafiltered{i}.acceleration(1),'r');
    hold on
    plot(i,datafiltered{i}.acceleration(2),'g');
    plot(i,datafiltered{i}.acceleration(3),'b');        
end
title('acceleration of filtered signal in x-, y- and z-direction (r,g,b)');




















