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
