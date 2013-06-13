function [ interval ] = obtain_boundaries_for_interpolation( dataOT, dataEMT )
interval = zeros(1,2);
numSensors = size(dataEMT,2);
vectorBoundaries = zeros(2,numSensors+1);
vectorBoundaries(:,1) = [dataOT{1}.TimeStamp; dataOT{end}.TimeStamp];
matrixHEMT = trackingdata_to_matrices(dataEMT,'CppCodeQuat');
for i = 1:numSensors
    pointEndTemp = [];
    pointEndTemp = find(1 ~= matrixHEMT{i}(4,4,:),1,'first')-1;
    if isempty(pointEndTemp)
        pointEndTemp = size(dataEMT,1);
    end
    vectorBoundaries(:,i+1) = [dataEMT{1,i}.TimeStamp, dataEMT{pointEndTemp,i}.TimeStamp];
end

interval = [max(vectorBoundaries(1,:)), min(vectorBoundaries(2,:))];

end

