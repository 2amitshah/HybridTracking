%% USE CASE
% Tracking data read-in.
% Author: Felix Achilles, July 2013

close all; clear all; clc

currentPath = which('read_raw_data_from_TrackingFusion.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.26_Measurements'];
testrow_name_EMT = 'EMT_2013_07_24_21_03_30_scissors';
testrow_name_OT = 'OpticalTrackingDirect_2';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

[dataOT, dataEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EMT, 'vDebug');

[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(dataEM, 'cpp');
[H_OT_to_OCS_cell] = trackingdata_to_matrices(dataOT, 'cpp');

EMfig = Plot_points(H_EMT_to_EMCS_cell,[],1,'x');
plotAuroraTable(EMfig)
plotAuroraVolume(EMfig)

OTfig = Plot_points(H_OT_to_OCS_cell,[],4,'o');