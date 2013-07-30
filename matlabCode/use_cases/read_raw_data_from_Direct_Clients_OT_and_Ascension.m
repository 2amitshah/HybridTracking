%% USE CASE
% Tracking data read-in. New format and Ascension system.
% Author: Felix Achilles, July 2013

close all; clear all; clc

currentPath = which('read_raw_data_from_TrackingFusion.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.30_Measurements'];
testrow_name_EMT = 'EMTrackingAscensionDirect';
testrow_name_OT = 'OpticalTrackingDirect';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

[dataOT, dataEM] = read_Direct_OpticalAndAscension(filenames_struct, [], [], 'vDebug');

[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(dataEM, 'ndi');
% Quaternion structure is [w, x, y, z] in Ascension, as repoted in
% ftp://ftp.ascension-tech.com/MANUALS/trakSTAR_Installation_and_Operation_Guide_%20RevD.pdf
% , Page 262. This is why mode 'ndi' is used (takes elements [2:4]).

[H_OT_to_OCS_cell] = trackingdata_to_matrices(dataOT, 'cpp');

EMfig = Plot_points(H_EMT_to_EMCS_cell,[],1,'x');

OTfig = Plot_points(H_OT_to_OCS_cell,[],4,'o');

% EM_frames = Plot_frames(H_EMT_to_EMCS_cell(1));