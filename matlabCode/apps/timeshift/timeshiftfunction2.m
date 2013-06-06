function [meanEuclideanDistance] = timeshiftfunction2(deltaT, dataEM, dataOT, numPoints)
%timeshiftfunction2 computes the mean euclidean distance of as many points
%as indicated in numPoints between dataEM and dataOT. It always compares
%the two points whose time stamps differ in a value deltaT. This function
%can be used to determine the correct timeshift deltaT between the two
%point sets when the meanEuclideanDistance is minimized.
%In order to minimize the meanEuclideanDistance the EM points and the OT points should be
%aligned as good as possible (using fixed points we can compute the
%needed transformation before calling this function)
%INPUT
%
%deltaT: 
%the current timeshift that is used to determine the euclidean distances
%
%dataEM:
%the data of the electromagnetic tracking
%
%dataOT:
%the data of the optical tracking which is shifted in time towards the data
%of the electromagnetic tracking
%
%numPoints:
%the amount of points of the optical tracking and the electromagnetic
%tracking that is used to determine the mean euclidean distance
%
%OUTPUT
%
%meanEuclideanDistance:
%the mean euclidean distance that we have between the points of the EM
%tracking and the OT tracking if we use a timeshift of deltaT
%

meanEuclideanDistance = 0;
min = 1;
if deltaT < 0
    min = abs(round(deltaT))+1;
end
max = numPoints;
if deltaT > 0
    max =numPoints-deltaT;
end
if min >=max
    meanEuclideanDistance = 10000000000000000000000000
end
for j = min:max
    meanEuclideanDistance = meanEuclideanDistance + sqrt( (dataOT{round(j+deltaT)}.position(1) - dataEM{j}.position(1))^2 + (dataOT{round(j+deltaT)}.position(2) - dataEM{j}.position(2))^2 + (dataOT{round(j+deltaT)}.position(3) - dataEM{j}.position(3))^2 );
end
if min < max
    meanEuclideanDistance = meanEuclideanDistance / (max-min)
end
deltaT
end