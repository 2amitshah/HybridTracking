%% USE CASE
% Distortion plot. Shows the distortion of the EM field
% Author: Felix Achilles, July 2013

close all; clear all; clc

y_computation;
close all;

filenames_struct.EMfiles = 'EMTracking_newsd1';
filenames_struct.OTfiles = 'OpticalTracking_newsd1';
[Fu, Fv, Fw] = distortion_new(filenames_struct,'vDebug', Y);