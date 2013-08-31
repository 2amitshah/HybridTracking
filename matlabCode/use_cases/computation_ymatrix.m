%% USE CASE
% Y Computation. Takes one continuous measurement without distortion, and
% computes the Y matrix
% Author: Amit Shah August 2013
% code from Felix Achilles, July 2013

close all; clear all; clc

currentPath = which('computation_ymatrix.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];
testrow_name_EMT = 'EM_Direct_2013_08_31';
testrow_name_OT = 'OT_Direct_2013_08_31';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

load('H_OT_to_EMT')

Y = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'ndi','static','vRelease');