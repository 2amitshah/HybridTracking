%% USE CASE
% Y Computation. Takes one continuous measurement without distortion, and
% computes the Y matrix
% Author: Felix Achilles, July 2013

close all; clear all; clc

currentPath = which('y_computation.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '08.16_Measurements'];
testrow_name_EMT = 'EMT_Direct_2013_08_16_15_23_54';
testrow_name_OT = 'OPT_Direct_2013_08_16_15_23_54';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

load('H_OT_to_EMT')

Y = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease');