%% USE CASE
% NDI TRACK Tracking data read-in.
% Author: Felix Achilles, September 2013

close all; clear all; clc

currentPath = which('read_raw_data_from_NDITrack_PolarisAndAurora.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep 'NDIRecordingsForPaper'];
testrow_name_EMT = 'EM_2013_09_18_16_45_000';
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