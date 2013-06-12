% polaris_to_aurora_v2 should be able to handle static data as well as data
% from dynamic recording to estimate the Y matrix.
% DOES NOT WORK YET (June 12, 2013)
%
% Author: Felix Achilles, May 2013

function Y = polaris_to_aurora_v2(path, H_OT_to_EMT, data_OT, data_EMT)
%Y = polaris_to_aurora(path, H_OT_to_EMT, data_EMT, data_OT)

%% data read in
% do preparation
% close all;
currentPath = which(mfilename);
if ~exist('path', 'var') || isempty(path)
    % read_TrackingFusion_files should be located in
    % HybridTracking\matlabCode\library\
    pathGeneral = fileparts(fileparts(fileparts(currentPath))); % pathGeneral is HybridTracking\
    file_path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];
end

if ~exist('H_OT_to_EMT', 'var') || isempty(H_OT_to_EMT)
    load(which('H_OT_to_EMT.mat'));
end

testrow_name_EMT = 'distorEMT';
testrow_name_OT = 'distorOT';

% get data for hand/eye calib
if ~exist('data_EMT', 'var')
    [data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
end
if ~exist('data_OT', 'var')
    [data_OT] = read_NDI_tracking_files(path, testrow_name_OT);
end

%prepare data
numPts = min(size(data_EMT,1),size(data_OT,1));
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

for i = 1:numPts

    %insert rotation into homogeneous matrix
    mat{1}(:,:,i) = quat2rot((data_EMT{i,1}.orientation(2:4))');

    mat{2}(:,:,i) = quat2rot((data_OT{i}.orientation(2:4))');

    %add translation
    mat{1}(:,:,i) = transl(data_EMT{i,1}.position') * mat{1}(:,:,i);

    mat{2}(:,:,i) = transl(data_OT{i}.position') * mat{2}(:,:,i);

end

%gHc = tracking_handEyeCalib(bHg, wHc)

% We want to have EMCS as world coordinates. World in Tsai's setup always
% means the grid. I will filp this to match EMT~Cam and OT~Marker.
for i = 1:numPts
    H_EMCS_to_EMT(:,:,i)    = inv(mat{1}(:,:,i)); %(=EMCS_to_EMT)
    H_EMT_to_EMCS(:,:,i)    = mat{1}(:,:,i);
    H_OT_to_OCS(:,:,i)      = mat{2}(:,:,i);      %(=OT_to_OCS)
    H_OCS_to_OT(:,:,i)      = inv(mat{2}(:,:,i));
end

%% averaging to find Y
Y_all = zeros(4,4,numPts);
% for i = 42:52
    Y_all(:,:,numPts) = H_EMT_to_EMCS(:,:,numPts) * H_OT_to_EMT * H_OCS_to_OT(:,:,numPts);
% end

Y_tmp = mean(Y_all(:,:,end),3);
Y(:,:) = Y_tmp(:,:,1);

end