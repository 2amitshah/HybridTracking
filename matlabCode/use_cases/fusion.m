%% USE CASE
% fusion: does a sensor fusion with the given data, without any
% interpolation (last available positions are used)
% Author: Nicola Leucht, July 2013

close all; clear all; clc
pathGeneral = fileparts(fileparts(fileparts(which(mfilename))));
if strcmp(pathGeneral, '')
    error('mfilename needs you to start the script with F5, not F9 or Ctrl+Enter')
end
path = [pathGeneral filesep 'measurements' filesep '08.16_Measurements'];

testrow_name_EM = 'EMT_Direct_2013_08_16_15_28_44';
testrow_name_OT = 'OPT_Direct_2013_08_16_15_28_44';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EM;
filenames_struct.OTfiles = testrow_name_OT;

kalmanfrequencyHz = 20;

% kalman_fusion_positions(filenames_struct, kalmanfrequencyHz, 'vDebug');

kalman_fusion_positions_DeviceTS(filenames_struct, kalmanfrequencyHz, 'vDebug');

