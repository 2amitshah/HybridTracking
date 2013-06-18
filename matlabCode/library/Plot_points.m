function plothandle = Plot_points(Frames_cell, plothandle, colorhandle)
% plothandle = Plot_points(Frames_cell, plothandle)
% Plot_points will plot all points of a given cell-dataset containing
% H-matrices from an arbitrary number of sensors. You can add a color
% index that will be used to access the colormap 'lines'.

numPts = size(Frames_cell{1},3);
numSen = size(Frames_cell,2);
if ~exist('plothandle', 'var') || isempty(plothandle)
    plothandle = figure;
end
if ~exist('colorhandle', 'var')
    colorhandle = 1;
end

%% plot position data
xaxes = cell(1,numSen);
yaxes = cell(1,numSen);
zaxes = cell(1,numSen);
emPoints = cell(1,numSen);

vectorEndPoints = zeros(1,numSen);

for j = 1:numSen
    pointEndTemp = find(1 ~= Frames_cell{1}(4,4,:),1,'first')-1;
    if (isempty(pointEndTemp))
        pointEndTemp = numPts;
    end
    vectorEndPoints(1,j) = pointEndTemp;
    for i = 1:vectorEndPoints(1,j)
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
    plot3(emPoints{j}(1,:), emPoints{j}(2,:), emPoints{j}(3,:), 'x', 'Color', c((colorhandle-1+j),:) );
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