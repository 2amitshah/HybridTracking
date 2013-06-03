function epsilon = objectiveFunctionDebug (x, mat, numPts, numSensors)

%result = zeros(numPts, numSensors);

        result= simulation(x,[mat{1,1}.C*mat{1,1}.x; mat{1,1}.constraints]);        
error1 = max(max(abs(result)));
% error2 = abs(norm(x(1:3))-1);
% aweight = 1;
% bweight = 10;
% epsilon = aweight*error1 + bweight*error2;
epsilon = error1;
disp(epsilon)

end