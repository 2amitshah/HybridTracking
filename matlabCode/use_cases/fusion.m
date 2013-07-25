%% USE CASE
% fusion: does a sensor fusion with the given data, without any
% interpolation (last available positions are used)
% Author: Nicola Leucht, July 2013

close all; clear all; clc
pathGeneral = fileparts(fileparts(fileparts(which(mfilename))));
path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];

testrow_name_EM = 'EMTrackingcont_1';
testrow_name_OT = 'OpticalTrackingcont_1';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EM;
filenames_struct.OTfiles = testrow_name_OT;

kalmanfrequencyHz = 5;

kalman_fusion_positions(filenames_struct, kalmanfrequencyHz, 'vDebug');

