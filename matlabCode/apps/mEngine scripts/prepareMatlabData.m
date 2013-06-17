% Load important .mat files
load('H_OT_to_EMT.mat');
load('Y.mat');
load('H_EMT1_to_EMT2.mat');
c = colormap('lines');
close(gcf);
[x,y,z] = sphere(20);
% load('H_EMT1_to_EMT3.mat');

% Execute environment plot
realtime_plot_figure = figure('Position', get(0,'ScreenSize'));
plotEnvironment(realtime_plot_figure, [], Y);
view(3);