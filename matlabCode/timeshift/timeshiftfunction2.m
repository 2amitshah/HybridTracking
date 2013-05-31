function [sum] = timeshiftfunction(deltaT, dataEM, dataOT, numPoints)


%numPoints = 5;


sum = 0;
min = 1;
if deltaT < 0
    min = abs(round(deltaT))+1;
end
max = numPoints;
if deltaT > 0
    max =numPoints-deltaT;
end
if min >=max
    sum = 10000000000000000000000000
end
for j = min:max
    sum = sum + sqrt( (dataOT{round(j+deltaT)}.Position(1) - dataEM{j}.Position(1))^2 + (dataOT{round(j+deltaT)}.Position(2) - dataEM{j}.Position(2))^2 + (dataOT{round(j+deltaT)}.Position(3) - dataEM{j}.Position(3))^2 );
end
if min < max
    sum = sum / (max-min)
end
deltaT
end