%PLOTENVIRONMENT Tracking Scenario Plot
% PLOTENVIRONMENT (plotHandle, H_OT_to_EMT,Y) plots the Optical and EM 
% tracking over a figure, where positions and orientation can be shown 
% with respect to both tracking systems. plothandle provides the handle 
% of the figure, H_OT_to_EMT is the OT to EMT matrix where rotation and
% translation can be computed. If this parameter is not provided, it finds 
% the best in the workspace.
% 
% Example
%     a = figure(1);
%     plotEnvironment(a);
%     hold on
%     line([-100 100], [-100 100], [0 -200]);
% 
% Created by Santiago Perez, June 2013
function plotEnvironment(plothandle, H_OT_to_EMT, Y)

% figHandles = findall(0,'Type','figure');
% if (find(figHandles == plothandle) > 0)
%     close(plothandle);
% end
% clc, close all, clear all

if ~exist('plothandle','var') 
    plothandle = figure;
end

if ~exist('H_OT_to_EMT', 'var')
    load(which('H_OT_to_EMT.mat'));
end

% Load transformation matrix and defines rotation & translation
if ~exist('Y', 'var')
    Y = polaris_to_aurora([],H_OT_to_EMT);
end

figure(plothandle);
hold on

%%AURORA TABLE
plotAuroraTable(plothandle);

%%AURORA VOLUME
plotAuroraVolume;

%%POLARIS
plotPolaris(plothandle, H_OT_to_EMT, Y);

% Options plot
axis image vis3d
set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')
camlight('headlight')
camlight('headlight')
lighting gouraud
xlabel('x');
ylabel('y');
zlabel('z');
view(65,30)
hold off

end

