function simulate_realtime_plot_and_fusion(file_path, file_prefixOT, file_prefixEMT)
% simulate_realtime_plot simulates data being read in by TrackingFusion.cpp
% and being sent to a matlab plot to show the concept and fps performance.
% Also the Environment (Polaris and Aurora device) is plotted.
%
% Data fusion means that OT data are taken if not occluded, if yes, EMT are
% mapped to OT position. If none is available, Red sphere is plotted as a
% warning.

% read measurements form disc and interpolate with 40Hz
[~, ~, data_EM_common, data_OT_common] = OT_and_common_EMT_at_synthetic_timestamps_V2(file_path, file_prefixEMT, file_prefixOT);

dataOT = data_OT_common;
dataEMT = data_EM_common;

% transform to homogenic 4x4 matrices
H_OT_to_OCS_cell = trackingdata_to_matrices(dataOT, 'CppCodeQuat');
H_EMT_to_EMCS_cell = trackingdata_to_matrices(dataEMT, 'CppCodeQuat');


%get Y
%this is apps\fusion\simulate_realtime_plot_and_fusion.m
currentPath = which('simulate_realtime_plot_and_fusion.m');
pathGeneral = fileparts(fileparts(fileparts(fileparts(currentPath))));
path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];
% Y = polaris_to_aurora(path, [], 'ndi');
load('Y')

% transform OT to EMCS coordinates
numOTpoints = size(H_OT_to_OCS_cell{1},3);
for i = 1:numOTpoints
    H_OT_to_EMCS_cell{1}(:,:,i) = Y*H_OT_to_OCS_cell{1}(:,:,i);
end

% plot environment
realtime_plot_figure = figure('Position', get(0,'ScreenSize'));
plotEnvironment(realtime_plot_figure, [], Y);

view(3)

% plot points into environment
Plot_points(H_OT_to_EMCS_cell, realtime_plot_figure, 1)
Plot_points(H_EMT_to_EMCS_cell, realtime_plot_figure, 2)

drawnow

% go through index column, plot points of given source
c = colormap('lines');
ot_ind = 1;
emt1_ind = 1;
emt2_ind = 1;
emt3_ind = 1;

for SensorIndex = 2*ones(1,numOTpoints)
    hold on
    switch SensorIndex;
        case 1 %OT sensor
            point = H_OT_to_EMCS_cell{1}(1:3,4,ot_ind);
            if exist('otObj', 'var'), delete(otObj); end
            otObj = plot3(point(1), point(2), point(3), 'o', 'Color', c(1,:) );
            ot_ind = ot_ind+1;
            
        case 2 %EMT1 sensor
            point = H_EMT_to_EMCS_cell{1}(1:3,4,emt1_ind);
            if exist('emt1Obj', 'var'), delete(emt1Obj); delete(cylinderObj); end
            emt1Obj = plot3(point(1), point(2), point(3), 'x', 'Color', c(2,:) );
            % plot a nice cylinder depicting the tool
            cylinderObj = Plot_cylinder(H_EMT_to_EMCS_cell{1}(:,:,emt1_ind));
            emt1_ind = emt1_ind+1;
            
        case 3 %EMT2 sensor
            point = H_EMT_to_EMCS_cell{SensorIndex-1}(1:3,4,emt2_ind);
            if exist('emt2Obj', 'var'), delete(emt2Obj); end
            emt2Obj = plot3(point(1), point(2), point(3), 'x', 'Color', c(3,:) );
            emt2_ind = emt2_ind+1;
            
        case 4 %EMT3 sensor
            point = H_EMT_to_EMCS_cell{SensorIndex-1}(1:3,4,emt3_ind);
            if exist('emt3Obj', 'var'), delete(emt3Obj); end
            emt3Obj = plot3(point(1), point(2), point(3), 'x', 'Color', c(4,:) );
            emt3_ind = emt3_ind+1;
    end
    hold off
    drawnow
end

end