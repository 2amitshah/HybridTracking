%% USE CASE
% X Calibration. Takes one continuous measurement without distortion, and
% computes the X matrix
% Author: Felix Achilles, September 2013

close all; clear all; clc

currentPath = which('X_Calibration.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack' filesep 'trus_recording'];
% for dynamic
% testrow_name_EMT = 'dynamic_EM_04_09_2013';
% testrow_name_OT = 'dynamic_OT_04_09_2013';

% for static
testrow_name_EMT = 'EM';
testrow_name_OT = 'OT';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

[H_OT_to_EMT_cell, H_OT_to_EMT, errors] = calibration_convex_optimization_motions(path, testrow_name_EMT, testrow_name_OT);