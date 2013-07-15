%% Author: Kanishka Sharma
% MATLAB script for post processing of raw data from gamma camera to create
% 3d LUT's ,to be used later for 3D reconstruction
%In order to use the complete script you need to download slicomatic for viewing results
% to do: write more efficient code :)
%%
close all;
clear all;
clc;

%%energy window for cobalt-57 
energy_window_low=110; 
energy_window_high=134;
%spacing  = 5;  %%step size of 150_150_150 measurements
spacing  = 1;  %%step size of 50_50_50 measurements

%read calibration matrix
fCali = 'homo.dat';
fName1 = fopen(fCali,'r');
cal_mat = fread(fName1,256*5,'double');
cal_mat = reshape(cal_mat,5,256);
fclose(fName1);

%measurement data
d = uigetdir(pwd, 'Select a folder'); %%get directory
files = dir(fullfile(d, '*.txt'));
array = zeros(256,size(files,1));
%LUT = zeros(256,31,31,31); %%uncomment for 150_150_150 volume
LUT50 = zeros(256,51,51,51);  %% initialize array for 50_50_50 volume

for n=1:size(files,1) %%number of measuremet files 
 file_name = files(n).name;
 [pathstr,name,ext] = fileparts(file_name);
 reg  = regexp(name,'_','split');
 AcqTime = reg{1,3};       
 AcqTime = str2double(char(AcqTime)); 
 X_pos = reg{1,4}; 
 X_pos = str2double(char(X_pos)); %%x-position in pos-table
 Y_pos =  reg{1,5};
 Y_pos = str2double(char(Y_pos)); %%y-position in pos-table
 Z_pos = reg{1,6};
 Z_pos = str2double(char(Z_pos)); %%z-position in pos-table
 tmp_name = strcat(name, '.txt');
 txt_file_name = fullfile(d,tmp_name); 
 fName2 = fopen(txt_file_name);
 raw_data = textscan(fName2,'%f %f','delimiter','');
 fclose(fName2);
 
 %%defining the 3D positions in the LUT-matrix of every pixel 
 position = cell(1,3);
 position{1,1} = X_pos;
 position{1,2} = Y_pos;
 position{1,3} = Z_pos;
 pos = cell2mat(position);
 x1 = (pos(1,1)/spacing)+1;
 x2 = (pos(1,2)/spacing)+1;
 x3 = (pos(1,3)/spacing)+1;
 
 raw_data =  cell2mat(raw_data); %%raw data from each measurement file
 energy_data = round(raw_data(:,2)); %%dividing each element with calibration factor
 pixel_temp = raw_data(:,1);
 
 for i=1:size(raw_data,1)
       energy_data(i) = raw_data(i,2) / cal_mat(1,(raw_data(i,1)+1)); %% energy calibration
 end
 
 energy_newData(:,1) = pixel_temp;
 energy_newData(:,2) = energy_data; 
 [energymat,X] = find((energy_newData(:,2)>energy_window_low)&...
                 (energy_newData(:,2)<energy_window_high));  %%apply energy window for cobalt 57 (110 - 134keV) on the diff channels
 energy(:,1) = pixel_temp(energymat); 
 energy(:,2) = energy_data(energymat);
 
   for i = 0:255
    [C,D] = find(energy(:,1)==i); %%finding the counts of different pixels and storing them in an array
    pixel_counts = sum(D);
    array((i+1),n) = pixel_counts*cal_mat(5,(i+1)); %%homogeneity correction per pixel
    LUT50((i+1),x1,x2,x3) = array((i+1),n);  %%4-D matrix containing all pixel counts in every position of the volume
   end
 
 clearvars raw_data energy_data pixel_temp energy_newData energymat energy %%clear variables for next iteration
end
 
save('LUT50.mat');

%%save per pixel LUT
%pixel_LUT_tmp = zeros(31,31,31); %% uncomment for 150_150_150 vol
%pixel_LUT_tmp = zeros(51,51,51);

%% write the 256 LUT's to .csv format
% for p = 1:256
%     pixel_LUT_tmp(:,:,:) = LUT50(p,:,:,:); %%pixelwise 3D matrix(LUT)
%     file = num2str(p);
%     d = '.csv';
%     full_file = strcat(file,d);
%     header_filename = file;
% %    header_volume = '150_150_150'; %%uncomment for 150_150_150 measurements
% %    header_volume_1 = '150';  %% measurement volume
% %    header_volume_2 = '150';
% %    header_volume_3 = '150';
%     header_volume_1 = '50';  %% measurement volume
%     header_volume_2 = '50';
%     header_volume_3 = '50';
%     header_time_acq = '7';  %%acquisition time
% %    header_spacing_x = '5'; %%uncomment for 150_150_150 measurements
% %    header_spacing_y = '5'; %%uncomment for 150_150_150 measurements
% %    header_spacing_z = '5'; %%uncomment for 150_150_150 measurements
%     header_spacing_x = '1';  %%spacing in x
%     header_spacing_y = '1';  %%spacing in y
%     header_spacing_z = '1';  %%spacing in z
% 
% 
% %% writing data to files: ONLY finally uncomment this part to save time before
% %     outid = fopen(full_file, 'w');
% %          fprintf(outid, '%s \n %s\n %s\n %s\n %s\n %s \n %s \n %s \n'...
% %             ,header_filename,header_volume_1,header_volume_2,header_volume_3,header_time_acq,header_spacing_x,header_spacing_y,header_spacing_z);
% %    dlmwrite(full_file,pixel_LUT_tmp,'delimiter',',','-append');
% %    fclose(outid);
% 
%     %% Interpolation only for the 150 x 150 x 150 LUT!
%     %maxVal = size(pixel_LUT_tmp,1);
%     %[X,Y,Z] = meshgrid(1:1:maxVal);
%     %dInterval is the grid size for the interpolated matrix
%     %dInterval = .2; %% approximation to resize matrix to spacing 1
%     %[XI,YI,Z1] = meshgrid(1:dInterval:maxVal);
%     %LUT150_interp(p,:,:,:) = interp3(X,Y,Z,pixel_LUT_tmp(:,:,:),XI,YI,Z1);
% 
% end

%% SLICEOMATIC VIEW
%%view volume - sliceomatic
%  x = 1:51; y = 1:51; z = 1:51;
%  [X,Y,Z] = meshgrid(x,y,z);
%  
%  %scatter3(X(:),Y(:),Z(:),51,pixel_LUT_tmp(:))
%  sliceomatic(pixel_LUT_tmp);

%sliceomatic();

