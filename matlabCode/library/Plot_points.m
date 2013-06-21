function plothandle = Plot_points(Frames_cell, plothandle, colorhandle)
% plothandle = Plot_points(Frames_cell, plothandle)
% Plot_points will plot all points of a given cell-dataset containing
% H-matrices from an arbitrary number of sensors. You can add a color
% index that will be used to access the colormap 'lines'.

numSen = size(Frames_cell,2);
if ~exist('plothandle', 'var') || isempty(plothandle)
    plothandle = figure;
end
if ~exist('colorhandle', 'var')
    colorhandle = 1;
end

%% plot position data
emPoints = cell(1,numSen);

vectorValidPoints = cell(1,numSen);

for j = 1:numSen
    pointsValidTemp = find(1 == Frames_cell{j}(4,4,:));
%     if (isempty(pointsValidTemp))
%         pointsValidTemp = 1:numPts;
%     end
    vectorValidPoints{j} = pointsValidTemp;
    emPoints{j} = zeros(3,numel(vectorValidPoints{j}));
    temp_indices = 1:numel(vectorValidPoints{j});
    %translation
    emPoints{j}(:,temp_indices) = Frames_cell{j}(1:3,4,vectorValidPoints{j});
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