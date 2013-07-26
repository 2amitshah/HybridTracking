%% USE CASE
% Y Computation. Takes one continuous measurement without distortion, and
% computes the Y matrix
% Author: Felix Achilles, July 2013

close all; clear all; clc

currentPath = which('y_computation.m');
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '07.16_Measurements'];
testrow_name_EMT = 'EMTracking_cont';
testrow_name_OT = 'OpticalTracking_cont';

filenames_struct.folder = path;
filenames_struct.EMfiles = testrow_name_EMT;
filenames_struct.OTfiles = testrow_name_OT;

load('H_OT_to_EMT')

Y = polaris_to_aurora_absor(filenames_struct, H_OT_to_EMT,'cpp','dynamic','vRelease');