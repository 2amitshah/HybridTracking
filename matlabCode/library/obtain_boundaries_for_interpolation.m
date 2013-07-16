function [ interval ] = obtain_boundaries_for_interpolation( dataOT, dataEMT )

numSensors = size(dataEMT,2);
vectorBoundaries = zeros(2,numSensors+1);
% timestamps from optical
matrixHOT_cell = trackingdata_to_matrices(dataOT,'CppCodeQuat');
% find index of last valid position
pointEndTemp = find(1 == matrixHOT_cell{1}(4,4,:),1,'last');
% find index of first valid position
pointStartTemp = find(1 == matrixHOT_cell{1}(4,4,:),1,'first');
% if no valid position exists, set pointEndTemp to number of positions
if isempty(pointEndTemp)
    pointEndTemp = size(dataEMT,1);
end
vectorBoundaries(:,1) = [dataOT{pointStartTemp,1}.TimeStamp; dataOT{pointEndTemp,1}.TimeStamp];
    
% timestamps from EM
matrixHEMT_cell = trackingdata_to_matrices(dataEMT,'CppCodeQuat');
for j = 1:numSensors
    % find index of last valid position
    pointEndTemp = find(1 == matrixHEMT_cell{j}(4,4,:),1,'last');
    % find index of first valid position
    pointStartTemp = find(1 == matrixHEMT_cell{j}(4,4,:),1,'first');
    % if no valid position exists, set pointEndTemp to number of positions
    if isempty(pointEndTemp)
        pointEndTemp = size(dataEMT,1);
    end
    vectorBoundaries(:,j+1) = [dataEMT{pointStartTemp,j}.TimeStamp; dataEMT{pointEndTemp,j}.TimeStamp];
end
% valid interval is between the latest (=max) of the start points and
% the earliest (=min) of the end points
interval = [max(vectorBoundaries(1,:)), min(vectorBoundaries(2,:))];

end

