function plothandle = Plot_frames(Frames_cell, plothandle)
numPts = size(Frames_cell{1},3);
numSen = size(Frames_cell,2);
if ~exist('plothandle', 'var') || isempty(plothandle)
    plothandle = figure;
end
%% plot position data
xaxes = cell(1,numSen);
yaxes = cell(1,numSen);
zaxes = cell(1,numSen);
emPoints = cell(1,numSen);

for j = 1:numSen
    for i = 1:numPts
        %em tracker
        %rotation
        xaxes{j}(i,:) = (Frames_cell{j}(1:3,1,i))';
        yaxes{j}(i,:) = (Frames_cell{j}(1:3,2,i))';
        zaxes{j}(i,:) = (Frames_cell{j}(1:3,3,i))';
        %translation
        emPoints{j}(:,i) = Frames_cell{j}(1:3,4,i);
    end
end
c = colormap('lines');

% plot all OT positions
figure(plothandle);
for j = 1:numSen
    
    hold on
    plot3(emPoints{j}(1,:), emPoints{j}(2,:), emPoints{j}(3,:), 'x', 'Color', c(j,:) );
    hold off


    % plot all OT orientations
    %x axis is red, y axis is green, z axis is blue
    hold on
    line([emPoints{j}(1,:); emPoints{j}(1,:)+30*xaxes{j}(:,1)'],...
        [emPoints{j}(2,:); emPoints{j}(2,:)+30*xaxes{j}(:,2)'],...
        [emPoints{j}(3,:); emPoints{j}(3,:)+30*xaxes{j}(:,3)'], 'LineWidth',3,'Color', [1 0 0]);
    hold off
    hold on
    line([emPoints{j}(1,:); emPoints{j}(1,:)+30*yaxes{j}(:,1)'],...
        [emPoints{j}(2,:); emPoints{j}(2,:)+30*yaxes{j}(:,2)'],...
        [emPoints{j}(3,:); emPoints{j}(3,:)+30*yaxes{j}(:,3)'], 'LineWidth',3,'Color', [0 1 0]);
    hold off
    hold on
    line([emPoints{j}(1,:); emPoints{j}(1,:)+30*zaxes{j}(:,1)'],...
        [emPoints{j}(2,:); emPoints{j}(2,:)+30*zaxes{j}(:,2)'],...
        [emPoints{j}(3,:); emPoints{j}(3,:)+30*zaxes{j}(:,3)'], 'LineWidth',3,'Color', [0 0 1]);
    hold off

end
title({'Sensor position in electromagnetic coordinate system EMCS',...
    'orientation is shown in XYZ = RGB'})
xlabel('x')
ylabel('y')
zlabel('z')
    
axis image vis3d

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')

view(3)
end