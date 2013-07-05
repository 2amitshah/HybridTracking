function mat = syntheticPositions (pointsToGather, probabilityFailure, noiseVariance)

if ~exist('pointsToGather','var')
     pointsToGather = 200;
end

if ~exist('probabilityFailure','var')
     probabilityFailure = 0.02;
end

if ~exist('noiseVariance','var')
     noiseVariance = 2;
end

close all;

% Limits EM
xmax = 170;
ymax = 260;
zmin = -120;
zmax = -500;
numPoints = 1500;
angleRotation = 45;
interval = [40802092906754 40858160673267]; % For example...

% Vectors
xvariation = linspace(-xmax,xmax,numPoints);
yvariation = ymax*sin(0.05*xvariation);
zvariation = linspace(zmin,zmax,numPoints);
timestampOriginal = round(linspace(interval(1),interval(2),numPoints));

% Position generation
position = [xvariation;yvariation;zvariation]';
matrixRotationZ = [ cos(angleRotation), sin(angleRotation), 0;
                    -sin(angleRotation), cos(angleRotation), 0;
                    0,0,1];
positionRotated = position * matrixRotationZ;

% Obtain random values
index = randperm(numPoints);
indexOrder = sort(index(1:pointsToGather));

positionRotatedDecimated = positionRotated(indexOrder,:) + noiseVariance*randn(numel(indexOrder),size(positionRotated,2));

timestamp = timestampOriginal(indexOrder);


mat = cell(pointsToGather,1);

for i = 1:size(mat,1)
    mat{i}.position = positionRotatedDecimated(i,:);
    mat{i}.TimeStamp = timestamp(i);
    mat{i}.valid = 1;
end

% Obtain random values
sensorFailing = round(pointsToGather*probabilityFailure);
nonValidVector = randperm(pointsToGather);
nonValidFirstPoints = nonValidVector(1:sensorFailing);

nonValidPoints = [];

for i = 1:numel(nonValidFirstPoints)
    valuesMissing = randi([2, 6]);
    pointInvalid = nonValidFirstPoints(i); % Yeah, it'd be a bit weird, but it was difficult to follow if I put everything together!
    nonValidPoints = [nonValidPoints pointInvalid-valuesMissing:pointInvalid+valuesMissing];
end

for i = 1:numel(nonValidPoints)
    mat{nonValidPoints(i)}.valid = 0;
end
all_pts = 1:pointsToGather;
valid_bool_vector = ~any(repmat(all_pts',1,numel(nonValidPoints)) == repmat(nonValidPoints,pointsToGather,1),2);

%plot
figure
plot3(positionRotatedDecimated(valid_bool_vector,1),positionRotatedDecimated(valid_bool_vector,2),positionRotatedDecimated(valid_bool_vector,3),'x');
axis vis3d image

end