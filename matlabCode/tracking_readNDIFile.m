%function [ data ] = tracking_readNDIFile( filename )
%% 
    if (true)
        clear all;
        close all;
        filename =  [pwd '/recordings/2013-22-01_01/calib_em_000.tsv'];
    end
    file =      fopen(filename);
    text =      textscan(file, '');
    
    fclose(file);
    
    
%end

