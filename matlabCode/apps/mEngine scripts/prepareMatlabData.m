clear all
close all
clc
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

trackercolor = 'red';
% prepare plot handles
positionOT_x = 0;
positionOT_y = 0;
positionOT_z = 0;
hold on; otObj = plot3(0,0,0, 'o', 'Color', trackercolor ); hold off;
set(otObj,'XDataSource','positionOT_x');
set(otObj,'YDataSource','positionOT_y');
set(otObj,'ZDataSource','positionOT_z');

% Plot a cylinder and obtain the object handles
Xcy_temp = zeros(2);
Ycy_temp = zeros(2);
Zcy_temp = zeros(2);
hold on
cylinderObj = surf(Xcy_temp, Ycy_temp, Zcy_temp, 'EdgeColor', 'none', 'FaceColor', 'y', 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud');
hold off
set(cylinderObj,'XDataSource','Xcy_temp');
set(cylinderObj,'YDataSource','Ycy_temp');
set(cylinderObj,'ZDataSource','Zcy_temp');

% plot a red sphere
xsp = x; ysp = y; zsp = z;
hold on
redsphere = surf(xsp,ysp,zsp, 'EdgeColor' , 'none', 'FaceColor', 'r', 'FaceLighting', 'gouraud');
hold off
set(redsphere,'XDataSource','xsp');
set(redsphere,'YDataSource','ysp');
set(redsphere,'ZDataSource','zsp');
set(redsphere,'Visible','off');

% Plot a cube and obtain the object handles
sizeCube = 60;
centerCube = [185,265,-sizeCube];
cubeVertices = repmat(sizeCube,8,3).*...
               [0 0 0; 0 1 0; 1 1 0; 1 0 0;...
                0 0 1; 0 1 1; 1 1 1; 1 0 1] + repmat(centerCube,8,1);
    
cubeFaces = [1 2 3 4; 2 6 7 3; 4 3 7 8;...
             1 5 8 4; 1 2 6 5; 5 6 7 8];
hold on
cubeObj=patch('Faces', cubeFaces,'Vertices',cubeVertices);
hold off
set(cubeObj,'edgecolor','k','FaceLighting', 'gouraud','facecolor','k');
