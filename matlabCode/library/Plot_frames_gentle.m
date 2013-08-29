function plothandle = Plot_frames_gentle(Frames_cell, plothandle, colorstring)
% plots every tenth orientation as a vector in x direction
numPts = size(Frames_cell{1},3);
numSen = size(Frames_cell,2);
if ~exist('plothandle', 'var') || isempty(plothandle)
    plothandle = figure;
end
if ~exist('colorstring', 'var') || isempty(colorstring)
    colorstring = 'k-0.5';
end
%% plot position data
xaxes = cell(1,numSen);
yaxes = cell(1,numSen);
zaxes = cell(1,numSen);
positions = cell(1,numSen);

for j = 1:numSen
    for i = 1:numPts
        %em tracker
        %rotation
        xaxes{j}(i,:) = (Frames_cell{j}(1:3,1,i))';
        yaxes{j}(i,:) = (Frames_cell{j}(1:3,2,i))';
        zaxes{j}(i,:) = (Frames_cell{j}(1:3,3,i))';
        %translation
        positions{j}(:,i) = Frames_cell{j}(1:3,4,i);
    end
end

% plot all OT positions
figure(plothandle);

for j = 1:numSen
    
    % plot all orientations
    %x axis is red, y axis is green, z axis is blue
    hold on
    arrow3(positions{j}(:,1:10:end)', positions{j}(:,1:10:end)'+30*xaxes{j}(1:10:end,:), colorstring, 1, 1)
    hold off

end
title({'Sensor position in reference frame.',...
    'orientation is shown as arrows in respective x direction'})

xlabel('x')
ylabel('y')
zlabel('z')

% axis([min(positions{j}(1,:)) max(positions{j}(1,:)) min(positions{j}(2,:)) max(positions{j}(2,:)) min(positions{j}(3,:)) max(positions{j}(3,:))])
% axis image vis3d
set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')

view(3)

end