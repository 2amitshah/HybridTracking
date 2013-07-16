


[opticalPoints_interp, OT_ts, emPointsFirstSensor_interp, EM_ts, realshift_nano] = interpolate_and_computeTimeShift(file_path, file_prefixOT, file_prefixEMT, datafreq)

%% averaging to find Y
Y_all = zeros(4,4,numPts);
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * H_OT_to_EMT * H_OCS_to_OT(:,:,i);
end

Y_tmp = mean(Y_all,3);
Y(:,:) = Y_tmp(:,:,1);
% sldkfjalsdjkfasd f
%sdfsdf
% dfasdf 
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
path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
testrow_name_EMT = 'EMTrackingcont_screwdriver_1';
testrow_name_OT = 'OpticalTrackingcont_screwdriver_1';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;


[~, ~, ~, Y_error] = distortion_new(filenames_struct,'vRelease');
% save('Fu.mat','Fu')
% save('Fv.mat','Fv')
% save('Fw.mat','Fw')

