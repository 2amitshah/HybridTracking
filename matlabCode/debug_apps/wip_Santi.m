%% 2013_06_10
close all;
clc

%this is debug_apps\wip_Felix.m
currentPath = which('wip_Santi.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];

file_prefixOT = 'OpticalTracking_cont';
file_prefixEMT = 'EMTracking_cont';


[dataOT, dataEMT] = read_TrackingFusion_files(path, file_prefixOT, file_prefixEMT);
dataOutput = synthetic_timestamps(dataEMT,[600000 601000],1e9);

%% 2013_06_12
clear all;
close all;
clc

pathGeneral = fileparts(fileparts(fileparts(which('wip_Santi.m'))));
path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];

load(which('H_OT_to_EMT.mat'));
Y = polaris_to_aurora(path, H_OT_to_EMT);

[data_OTraw, data_EMTraw,~,~] = read_TrackingFusion_files(path);

H_EMT_to_EMCSrawcell = trackingdata_to_matrices(data_EMTraw,'CppCodeQuat');
H_OT_to_OCSrawcell = trackingdata_to_matrices(data_OTraw,'CppCodeQuat');
interval = obtain_boundaries_for_interpolation(data_OTraw, data_EMTraw);
data_OT = synthetic_timestamps(data_OTraw,interval,30);
data_EMT = synthetic_timestamps(data_EMTraw,interval,30);

H_EMT_to_EMCS = trackingdata_to_matrices(data_EMT,'CppCodeQuat');
H_OT_to_OCS = trackingdata_to_matrices(data_OT,'CppCodeQuat');
a = figure;
subplot(1,2,1), EMTrawhandle = Plot_points(H_EMT_to_EMCSrawcell,a);
subplot(1,2,2), EMThandle = Plot_points(H_EMT_to_EMCS,a);