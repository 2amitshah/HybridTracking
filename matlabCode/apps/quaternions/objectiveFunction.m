function epsilon = objectiveFunction (x, mat, numPts, numSensors)

% Evaluates every solution by calling the simulation method. Provides some
% weight and enables the possibility of adding more weights and/or constraints
result = zeros(numPts, numSensors);
for i = 1:numPts-1
    for j = 1:numSensors
        result(i,j) = simulation(x,mat{i,j}.equations);        
    end
end
error1 = max(max(abs(result)));

epsilon = error1;
disp(epsilon)

end