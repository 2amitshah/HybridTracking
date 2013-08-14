%% USE CASE
% Tracking data read-in. New format including Error Value and DeviceTimestamp.
% Author: Felix Achilles, August 2013

close all; clear all; clc

currentPath = which(mfilename);
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '08.14_Measurements'];
testrow_name_EMT = 'EMTrackingDirect';
testrow_name_OT = 'OpticalTrackingDirect';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

[dataOT, dataEM] = read_Direct_NDI_PolarisAndAurora(filenames_struct, [], [], 'vDebug');

[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(dataEM, 'cpp');
[H_OT_to_OCS_cell] = trackingdata_to_matrices(dataOT, 'cpp');

EMfig = Plot_points(H_EMT_to_EMCS_cell,[],1,'x');
OTfig = Plot_points(H_OT_to_OCS_cell,[],4,'o');
