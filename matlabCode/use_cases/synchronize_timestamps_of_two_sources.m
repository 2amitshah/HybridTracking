%% USE CASE
% Tracking data read-in. Then a point-could-fit is used to guess the
% timestamp-offset.
% Author: Felix Achilles, July 2013

close all; clear all;

currentPath = which('read_raw_data_from_TrackingFusion.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.30_Measurements'];
testrow_name_EMT = 'EMTrackingAscensionDirect';
testrow_name_OT = 'OpticalTrackingDirect';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

sync_from_file(filenames_struct,'vDebug')