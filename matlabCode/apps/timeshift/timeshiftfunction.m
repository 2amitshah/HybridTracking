function [sum] = timeshiftfunction(deltaT, dataEM, dataOT, interOptPos,numPoints)


%numPoints = 5;
for i = 1:numPoints-1

    alpha = ((dataEM{i}.TimeStamp + deltaT) - dataOT{i}.TimeStamp) / (dataOT{i+1}.TimeStamp - dataOT{i}.TimeStamp);
    for j = 1:3
        interOptPos{i}.Position(j) = (1-alpha) * dataOT{i}.Position(j) + alpha * dataOT{i+1}.Position(j);
    end
    interOptPos{i}.TimeStamp = (1-alpha) * dataOT{i}.TimeStamp + alpha * dataOT{i+1}.TimeStamp;
end

sum = 0;
for i = 1:numPoints-1
    sum = sum + sqrt( (interOptPos{i}.Position(1) - dataEM{i}.Position(1))^2 + (interOptPos{i}.Position(1) - dataEM{i}.Position(1))^2 + (interOptPos{i}.Position(1) - dataEM{i}.Position(1))^2 );
end
sum
deltaT
end