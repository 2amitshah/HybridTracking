error('DO NOT START A WIP SCRIPT WITH F5! USE CTRL+ENTER TO RUN DISTINCT SECTIONS.')

%% start of wip (work in progress) script
[opticalPoints_interp, OT_ts, emPointsFirstSensor_interp, EM_ts, realshift_nano] = interpolate_and_computeTimeShift(file_path, file_prefixOT, file_prefixEMT, datafreq)

%% averaging to find Y
Y_all = zeros(4,4,numPts);
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * H_OT_to_EMT * H_OCS_to_OT(:,:,i);
end

Y_tmp = mean(Y_all,3);
Y(:,:) = Y_tmp(:,:,1);

%% 2013_06_04
% cd '.\apps'
path = '..\..\measurements\06.04_Measurements\';
otfile = 'cont_OpticalTracking';
emfile = 'cont_EMTracking';
test=interpolate_and_computeTimeShift(path,otfile,emfile,100);

%% 2013_06_07
close all
clear all
clc
% read measurements form disc
[dataOT, dataEMT] = read_TrackingFusion_files;

% transform to homogenic 4x4 matrices
H_OT_to_OCS_cell = trackingdata_to_matrices(dataOT, 'CppCodeQuat');
H_EMT_to_EMCS_cell = trackingdata_to_matrices(dataEMT, 'CppCodeQuat');

% break
% H_EMT_to_EMCS1 = H_EMT_to_EMCS_cell{1};
% H_EMT_to_EMCS2 = H_EMT_to_EMCS_cell{2};
% break
% H_EMT_to_EMCS1 = H_EMT_to_EMCS1(:,:,1:300);
% H_EMT_to_EMCS2 = H_EMT_to_EMCS2(:,:,1:300);

% H_EMT_to_EMCS_cell{1} = H_EMT_to_EMCS1;
% H_EMT_to_EMCS_cell{2} = H_EMT_to_EMCS2;

% plot locations
EMCS_plot_handle = Plot_points(H_EMT_to_EMCS_cell);
OCS_plot_handle = Plot_points(H_OT_to_OCS_cell);

%this is debug_apps\wip_Felix.m
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];
% get Y 
Y = polaris_to_aurora(path);

% transform OT to EMCS coordinates
numOTpoints = size(H_OT_to_OCS_cell{1},3);

for i = 1:numOTpoints
    H_OT_to_EMCS_cell{1}(:,:,i) = Y*H_OT_to_OCS_cell{1}(:,:,i);
end

% plot ot frame into EMCS plot
Plot_points(H_OT_to_EMCS_cell, EMCS_plot_handle)
% break
%nice!
plotEnvironment(EMCS_plot_handle,[],Y)

%% 2013_06_10
close all

%this is debug_apps\wip_Felix.m
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '06.11_Measurements'];

file_prefixOT = 'OpticalTracking_cont_1st';
file_prefixEMT = 'EMTracking_cont_1st';

simulate_realtime_plot(path, file_prefixOT, file_prefixEMT)

%% 2013_06_12
close all

%this is debug_apps\wip_Felix.m
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
% path = [pathGeneral filesep 'measurements' filesep '06.11_Measurements'];
path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];


file_prefixOT = 'OpticalTrackingcont_1';
file_prefixEMT = 'EMTrackingcont_1';

simulate_realtime_plot_and_fusion(path, file_prefixOT, file_prefixEMT)

%% 2013_06_17
close all

%this is debug_apps\wip_Felix.m
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];

% [H_OT_to_EMT, errors] = calibration_OT_to_common_EMT(path, 'EM_', 'OT_')
% [H_OT_to_EMT, errors] = calibration_OT_to_common_EMT(path, 'distorEMT_', 'distorOT_')
% the only way it works nicely:
[H_OT_to_EMT, errors] = calibration_OT_to_common_EMT %default path testmfrom_NDItrack and hybridEMT and hybridOT

%% 2013_06_18
close all
% I want to calculate Y and compare cpp and ndi result
%this is debug_apps\wip_Felix.m
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '06.18_Measurements'];
Y_ndi = polaris_to_aurora(path, [], 'ndi');

path = [pathGeneral filesep 'measurements' filesep '06.18_Measurements' filesep 'cpp_measurements'];
Y_cpp = polaris_to_aurora(path, [], 'cpp');

% test equalness
Y_ndi/Y_cpp

Y_cpp/Y_ndi

% watch different outcome
testfig=figure;
plotEnvironment(testfig,[],Y_ndi)
hold on
plotEnvironment(testfig,[],Y_cpp)
hold off

%% 2013_06_19 and 2013_06_24
close all
% I want to calculate Y from dynamic cpp data
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];

% and debug 'OT_common_EMT_at_synthetic_timestamps'
testrow_name_EMT = 'EMTrackingcont_1';
testrow_name_OT = 'OpticalTrackingcont_1';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

Y_cpp_dyn = polaris_to_aurora(filenames_struct, [], 'cpp', 'dynamic');

% and now compare that to the static result
% path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
Y_cpp_stat = polaris_to_aurora(path, [], 'cpp', 'static');

% test equalness
Y_cpp_stat/Y_cpp_dyn
%Result:
%    [1.0000    0.0034    0.0012   -1.6584
%    -0.0035    1.0000   -0.0105   -3.1397
%    -0.0012    0.0104    0.9999    1.7775
%          0         0         0    1.0000]

Y_cpp_dyn/Y_cpp_stat

% watch different outcome
testfig=figure;
plotEnvironment(testfig,[],Y_cpp_dyn)
hold on
plotEnvironment(testfig,[],Y_cpp_stat)
hold off
%Result: Looks pretty enough!

%% 2013_06_24
close all
% I want to perform the deviation/distortion/accuracy measure on some Data,
% store the results and compare them to the Word document.

% Also i want to have a Document that describes the Form of each of our
% curves. It will be a txt file located in /measurements.

% plot the form of a dynamic recording
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '05.29_Measurements'];
testrow_name_EMT = 'cont_EMTracking';
testrow_name_OT = 'cont_OpticalTracking';
% [H_commonEMT_to_EMCS] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT, testrow_name_OT, 20);
% H_commonEMT_to_EMCS_cell{1}=H_commonEMT_to_EMCS;
% fig=Plot_points(H_commonEMT_to_EMCS_cell);

Y_cpp_dyn = polaris_to_aurora(path, [], 'cpp', 'dynamic');


%% 2013_06_26
close all
% I want to calculate Y from dynamic cpp data
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];

% and debug 'OT_common_EMT_at_synthetic_timestamps'
testrow_name_EMT = 'EMTrackingcont_1';
testrow_name_OT = 'OpticalTrackingcont_1';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

Y_cpp_dyn = polaris_to_aurora(filenames_struct, [], 'cpp', 'dynamic');
%
% NEW CODE!
[dataOT, dataEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT);

interval(1) = dataOT{1}.TimeStamp;
interval(2) = dataOT{end}.TimeStamp;
[H_X_to_XBase_interp] = frame_interpolation(dataEM(:,1), interval, 20, 'cpp');
testfig = figure;
wrapper{1} = H_X_to_XBase_interp;%(:,:,50:90);
Plot_points(wrapper,testfig);
% Plot_frames(wrapper,testfig);

[H_X_to_XBase_cell]=trackingdata_to_matrices(dataEM(:,1),'CppCodeQuat');
Plot_points(H_X_to_XBase_cell,testfig,3); %red
% Plot_frames(H_X_to_XBase_cell,testfig,3); %red


%% Try out EKF
% Example:
%
n=3;      %number of state
q=0.1;    %std of process 
r=0.1;    %std of measurement
Q=q^2*eye(n); % covariance of process
R=r^2;        % covariance of measurement  
f=@(x)[x(2);x(3);0.05*x(1)*(x(2)+x(3))];  % nonlinear state equations
h=@(x)x(1);                               % measurement equation
s=[0;0;1];                                % initial state
x=s+q*randn(3,1); %initial state          % initial state with noise
P = eye(n);                               % initial state covraiance
N=20;                                     % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual
zV = zeros(1,N);
for k=1:N
  z = h(s) + r*randn;                     % measurments
  sV(:,k)= s;                             % save actual state
  zV(k)  = z;                             % save measurment
  [x, P] = ekf(f,x,P,h,z,Q,R);            % ekf 
  xV(:,k) = x;                            % save estimate
  s = f(s) + q*randn(3,1);                % update process 
end
for k=1:3                                 % plot results
  subplot(3,1,k)
  plot(1:N, sV(k,:), '-', 1:N, xV(k,:), '--')
end
%

%% 2013_07_11
% process the hopefully final distortion recordings
close all

currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.11_Measurements'];

load('H_OT_to_EMT')


%testrun
testrow_name_EMT = 'EMTracking_testrun';
testrow_name_OT = 'OpticalTracking_testrun';
distortion_nicola(path, H_OT_to_EMT, testrow_name_EMT, testrow_name_OT);
%%
%first try with a big recording

testrow_name_EMT = 'EMTracking_firstVolume';
testrow_name_OT = 'OpticalTracking_firstVolume';
distortion_nicola(path, H_OT_to_EMT, testrow_name_EMT, testrow_name_OT);

%% 2013_07_12
% write distortion correction
close all

currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.11_Measurements'];
testrow_name_EMT = 'EMTracking_testrun';
testrow_name_OT = 'OpticalTracking_testrun';

load('H_OT_to_EMT')

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

Y_cpp_dyn = polaris_to_aurora(filenames_struct, [], 'cpp', 'dynamic', 'vRelease');

%%%%
%correction function
%%%%
[data_OT, data_EMT, ~,~] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 1);
data_EMT_corrected = distortion_correction(data_EMT);
%%%%

%test new Y calculation
distortion_nicola(path, H_OT_to_EMT, testrow_name_EMT, testrow_name_OT);

% show effect of new Y on Boxplot
EM_accuracy_acquisition_withwithout_correction

%% 2013_07_16
% save new Fu, Fv, Fw interpolators

currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.16_Measurements'];
testrow_name_EMT = 'EMTracking_cont';
testrow_name_OT = 'OpticalTracking_cont';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;
load('H_OT_to_EMT')
 Y = polaris_to_aurora(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease');
filenames_struct.EMfiles = 'EMTracking_newsd1';
filenames_struct.OTfiles = 'OpticalTracking_newsd1';
[~, ~, ~, Y_error] = distortion_new(filenames_struct,'vRelease', Y);
% save('Fu.mat','Fu')
% save('Fv.mat','Fv')
% save('Fw.mat','Fw')

%% 2013_07_18
% save new Fu, Fv, Fw interpolators
close all
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.16_Measurements'];
testrow_name_EMT = 'EMTracking_cont';
testrow_name_OT = 'OpticalTracking_cont';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

load('H_OT_to_EMT')
% kalman_fusion_positions(path,testrow_name_EMT,testrow_name_OT,H_OT_to_EMT,10,'vDebug')

Y = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease');
%%
filenames_struct.EMfiles = 'EMTracking_newsd1';
filenames_struct.OTfiles = 'OpticalTracking_newsd1';
[Fu, Fv, Fw] = distortion_new(filenames_struct,'vRelease', Y);
% save('Fu.mat','Fu')
% save('Fv.mat','Fv')
% save('Fw.mat','Fw')

%% 2013_07_19
% correct screwdriver recording with previously generated Fu, Fv, Fw

close all
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.16_Measurements'];
testrow_name_EMT = 'EMTracking_newsd2';
testrow_name_OT = 'OpticalTracking_newsd2';

frequency = 20;
[~, ~, data_EMT, data_OT] = OT_common_EMT_at_synthetic_timestamps(path, testrow_name_EMT,testrow_name_OT, frequency, 'vRelease');
data_EMT_corrected = distortion_correction(data_EMT, Fu, Fv, Fw);


% Relevant matrix for computing transformations
[H_EMT_to_EMCS] =           trackingdata_to_matrices(data_EMT, 'CppCodeQuat');
[H_EMT_to_EMCS_corrected] = trackingdata_to_matrices(data_EMT_corrected, 'CppCodeQuat');
[H_OT_to_OCS] =             trackingdata_to_matrices(data_OT, 'CppCodeQuat');

H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};
H_EMT_to_EMCS_corrected = H_EMT_to_EMCS_corrected{1};
H_OT_to_OCS = H_OT_to_OCS{1,1};

% calculate where EM tracker should be
load('H_OT_to_EMT')
numPts = size(H_EMT_to_EMCS, 3);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
normPosition = [];
normPositionCorrected = [];

for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS / H_OT_to_EMT; %data_EM_common_by_OT
    
    if(data_OT{i}.valid == 1 && data_EMT{i}.valid == 1)
        H_diff_EMT_to_EMCS(:,:) = (H_EMT_to_EMCS(:,:,i))\H_EMT_to_EMCS_by_OT(:,:,i);
        normPosition = [normPosition norm(H_diff_EMT_to_EMCS(1:3,4))];
        H_diff_EMT_to_EMCS_corrected(:,:) = (H_EMT_to_EMCS_corrected(:,:,i))\H_EMT_to_EMCS_by_OT(:,:,i);
        normPositionCorrected = [normPositionCorrected norm(H_diff_EMT_to_EMCS_corrected(1:3,4))];
    end
end
figure;
boxplot([normPosition', normPositionCorrected'],'label',{'original positions','corrected positions'});
title('whadduuuuup')

%% 2013_07_22
% compare linear to homogenuous interpolation
% --> result: both yield the same/similar results as well in position as in
% orientation. Error found in Y or in ICP calculation does not arise from
% quaternion interpolation error as suspected.

close all

interpolation_frequency = 20;

currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.16_Measurements'];

% and debug 'OT_common_EMT_at_synthetic_timestamps'
testrow_name_EMT = 'EMTracking_cont';
testrow_name_OT = 'OpticalTracking_cont';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

% Y_cpp_dyn = polaris_to_aurora(filenames_struct, [], 'cpp', 'dynamic');
%
% NEW CODE!
[dataOT, dataEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT);
interval = obtain_boundaries_for_interpolation(dataOT, dataEM);
[H_EMT_to_EMCS_interp] = frame_interpolation(dataEM(:,1), interval, interpolation_frequency, 'cpp');


dataEM_interp = synthetic_timestamps( dataEM(:,1), interval, interpolation_frequency);
H_dataEM_interp_cell = trackingdata_to_matrices( dataEM_interp, 'cpp' );

position_testfig = figure;
wrapper{1} = H_EMT_to_EMCS_interp;%(:,:,50:90);
Plot_points(wrapper,position_testfig, 1, 'x'); % new interpolated data as blue x
Plot_points(H_dataEM_interp_cell,position_testfig, 3, 'o'); % old interpolated data as red o
title('Interpolated EMT data, old linear Interpolation as red o, new homogenuous transformation interpolation as blue x')

orientation_testfig1 = figure;
Plot_frames(wrapper,orientation_testfig1); 
title('Orientation of the frames of new homogenuous transformation interpolation')
orientation_testfig2 = figure;
Plot_frames(H_dataEM_interp_cell,orientation_testfig2);
title('Orientation of the frames of old linear interpolation')

%% 2013_07_30

close all

interpolation_frequency = 20;

currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.30_Measurements'];

% and debug 'OT_common_EMT_at_synthetic_timestamps'
testrow_name_EMT = 'EMTrackingAscensionDirect_2';
testrow_name_OT = 'OpticalTrackingDirect_2';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

[H_OT_to_EMT, errors] = dynamic_calibration_OT_to_common_EMT(filenames_struct, [], [], 'vDebug');

%% 2013_08_14
close all
interpolation_frequency = 20;
currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.30_Measurements'];




n=3;      %number of state
q=0.1;    %std of process 
r=0.1;    %std of measurement
Q=q^2*eye(n); % covariance of process
R=r^2;        % covariance of measurement  
f=@(x)[x(2);x(3);0.05*x(1)*(x(2)+x(3))];  % nonlinear state equations
h=@(x)x(1);                               % measurement equation
s=[0;0;1];                                % initial state
x=s+q*randn(3,1); %initial state          % initial state with noise
P = eye(n);                               % initial state covraiance
N=20;                                     % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual
zV = zeros(1,N);
for k=1:N
  z = h(s) + r*randn;                     % measurments
  sV(:,k)= s;                             % save actual state
  zV(k)  = z;                             % save measurment
  [x, P] = ukf(f,x,P,h,z,Q,R);            % ekf 
  xV(:,k) = x;                            % save estimate
  s = f(s) + q*randn(3,1);                % update process 
end
for k=1:3                                 % plot results
  subplot(3,1,k)
  plot(1:N, sV(k,:), '-', 1:N, xV(k,:), '--')
end

%% 2013_08_26
close all

RadFile = fopen('C:\Users\DCUser_02\Dropbox\Masterarbeit\messungenIFLradiation\calibrated_20130715_1433_40_1_1_1.dat');
RadData = fread(RadFile, [16,16], 'double');
% indicesnonZero = find(RadData);
% RadData_filtered = RadData(indicesnonZero);
% max(RadData_filtered)
RawData_figure = figure;
imagesc(RadData)
colorbar
axis image

%% 2013_08_29
close all; clc;
test = ukf_fusion_separate_kalmans_updatefcn;

%% 2013_09_18
% Y Computation. Takes one continuous measurement without distortion, and
% computes the Y matrix

close all; clear all; clc

currentPath = which('y_computation.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack' filesep 'CalibrationForPaper'];
testrow_name_EMT = 'EM_';
testrow_name_OT = 'OT_';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

load('H_OT_to_EMT')

Y = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'ndi','static','vDebug','device');
%%
% Fusion
close all; clear all; clc
[KalmanDataMaster, KalmanDataOT, KalmanDataEM] = ukf_fusion_OT_groundtruth;

%%
% Tracking data read-in. New format including Error Value and DeviceTimestamp.
% Author: Felix Achilles, August 2013

close all; clear all; clc

currentPath = which('wip_Felix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '09.19_Measurements'];
testrow_name_EMT = 'EMT_Direct_2013_09_19_14_03_17';
testrow_name_OT = 'OPT_Direct_2013_09_19_14_03_17';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

[dataOT, dataEM] = read_Direct_NDI_PolarisAndAurora(filenames_struct, 'vDebug');

[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(dataEM, 'cpp');
[H_OT_to_OCS_cell] = trackingdata_to_matrices(dataOT, 'cpp');

EMfig = Plot_points(H_EMT_to_EMCS_cell,[],1,'x');
OTfig = Plot_points(H_OT_to_OCS_cell,[],4,'o');

%% 2013_09_19
% NDI TRACK Tracking data read-in.
% Author: Felix Achilles, September 2013

close all; clear all; clc

currentPath = which('read_raw_data_from_NDITrack_PolarisAndAurora.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep 'NDIRecordingsForPaper'];
testrow_name_EMT = 'EM_2013_09_18_16_45_CalibrationMotions';
% testrow_name_OT = 'OpticalTrackingDirect_2';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
% filenames_struct.OTfiles = testrow_name_OT;

[ dataEM, data_raw, info ] = read_NDI_tracking_files( path, testrow_name_EMT, 'dynamic', 'Aurora');

[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(dataEM, 'cpp');
% [H_OT_to_OCS_cell] = trackingdata_to_matrices(dataOT, 'cpp');

EMfig = Plot_points(H_EMT_to_EMCS_cell,[],1,'x');
plotAuroraTable(EMfig)
plotAuroraVolume(EMfig)

% OTfig = Plot_points(H_OT_to_OCS_cell,[],4,'o');






