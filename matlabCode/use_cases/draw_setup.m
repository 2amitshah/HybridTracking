%% USE CASE
% Draw setup. Reads in X and Y matrix and calls our nice plot functions to
% show NDI Polaris and NDI Aurora withEM tracking volume.
% Author: Felix Achilles, August 2013

close all; clear all; clc

load('Y')
load('H_OT_to_EMT')
plothandle = figure;

% plotEnvironment([],H_OT_to_EMT, Y)
% 
% H_EMT_to_EMCS_tmp = [   1 0 0 0
%                         0 0 1 0
%                         0 1 0 -200];             
% opticalPoints_EMCS = [0; 0; -200];
% i = 1;
% [Xsp, Ysp, Zsp] = sphere(20);
% 
% Plot_cylinder(H_EMT_to_EMCS_tmp)
% 
% hold on
% surf(Xsp+opticalPoints_EMCS(1,i), Ysp+opticalPoints_EMCS(2,i), Zsp+opticalPoints_EMCS(3,i), 'EdgeColor', 'none', 'FaceColor', [.9 .9 .9], 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
% hold off
% %plot white stick that connects OT to the tool
% hold on
% line([opticalPoints_EMCS(1,i);opticalPoints_EMCS(1,i)+20*H_EMT_to_EMCS_tmp(1,2)],...
%     [opticalPoints_EMCS(2,i);opticalPoints_EMCS(2,i)+20*H_EMT_to_EMCS_tmp(2,2)],...
%     [opticalPoints_EMCS(3,i);opticalPoints_EMCS(3,i)+20*H_EMT_to_EMCS_tmp(3,2)], 'Color', 'w', 'LineWidth', 3)
% hold off
% 
% axis off

%%AURORA TABLE
plotAuroraTable(plothandle);

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

axis off