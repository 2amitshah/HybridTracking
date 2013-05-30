function epsilon = objectiveFunction (x, mat, numPts, numSensors)

result = zeros(numPts, numSensors);
for i = 1:numPts-1
    for j = 1:numSensors
        result(i,j) = simulation(x,mat{i,j}.equations);        
    end
end
error1 = max(max(abs(result)));
% error2 = abs(norm(x(1:3))-1);
% aweight = 1;
% bweight = 10;
% epsilon = aweight*error1 + bweight*error2;
epsilon = error1;
disp(epsilon)

end