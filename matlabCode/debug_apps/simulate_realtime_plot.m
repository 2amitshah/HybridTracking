function simulate_realtime_plot(file_path, file_prefixOT, file_prefixEMT)
% simulate_realtime_plot simulates data being read in by TrackingFusion.cpp
% and being sent to a matlab plot to show the concept and fps performance.
% Also the Environment (Polaris and Aurora device) is plotted.

% read measurements form disc
[dataOT, dataEMT] = read_TrackingFusion_files(file_path, file_prefixOT, file_prefixEMT);
numOTPts = size(dataOT,1);
numEMPts = size(dataEMT,1);
numEMSensors = size(dataEMT,2);

% sort timestamps
TS_OT = zeros(numOTPts,1);
for i = 1:numOTPts
    TS_OT(i,1)=dataOT{i}.TimeStamp;
    TS_OT(i,2)=1;
end
TS_EM = zeros(numEMPts,numEMSensors);
for i = 1:numEMPts
    for j = 1:numEMSensors
        if ~isempty(dataEMT{i,j})
            TS_EM(i,j*2-1)=dataEMT{i,j}.TimeStamp;
            TS_EM(i,j*2)=j+1;
        end
    end
end
%  rearrange EM timestamps
% TS_EM_new = zeros(numEMPts*numEMSensors,2);
TS_EM_new = [];
index_of_TS_EM_new = 0;
for j = 1:numEMSensors
    index_of_last_available_pt = find(TS_EM(:,2*j), 1, 'last');
    TS_EM_new = [TS_EM_new; TS_EM(1:index_of_last_available_pt,(2*j-1):2*j)];
%     start_index = index_of_TS_EM_new+1;
%     index_of_TS_EM_new = index_of_TS_EM_new + index_of_last_available_pt - 1;
%     TS_EM_new(start_index:index_of_TS_EM_new,:)=TS_EM(1:index_of_last_available_pt,(2*j-1):2*j);
end
index_of_last_available_pt = find(TS_EM_new(:,1), 1, 'last');
TS_EM_new = TS_EM_new(1:index_of_last_available_pt,:);
TS_all = [TS_OT; TS_EM_new];
[TS_all(:, 1), sort_indexes] = sort(TS_all(:, 1));
TS_all(:, 2) = TS_all(sort_indexes,2);

% go through index column, plot points of given source
% transform to homogenic 4x4 matrices
H_OT_to_OCS_cell = trackingdata_to_matrices(dataOT);
H_EMT_to_EMCS_cell = trackingdata_to_matrices(dataEMT);

% break
% H_EMT_to_EMCS1 = H_EMT_to_EMCS_cell{1};
% H_EMT_to_EMCS2 = H_EMT_to_EMCS_cell{2};
% break
% H_EMT_to_EMCS1 = H_EMT_to_EMCS1(:,:,1:300);
% H_EMT_to_EMCS2 = H_EMT_to_EMCS2(:,:,1:300);

% H_EMT_to_EMCS_cell{1} = H_EMT_to_EMCS1;
% H_EMT_to_EMCS_cell{2} = H_EMT_to_EMCS2;

% plot locations
EMCS_plot_handle = Plot_points(H_EMT_to_EMCS_cell);
OCS_plot_handle = Plot_points(H_OT_to_OCS_cell);

%this is debug_apps\wip_Felix.m
currentPath = which(mfilename);
pathGeneral = fileparts(fileparts(fileparts(currentPath)));
path = [pathGeneral filesep 'measurements' filesep '06.07_Measurements'];
% get Y 
Y = polaris_to_aurora(path);

% transform OT to EMCS coordinates
numOTpoints = size(H_OT_to_OCS_cell{1},3);

for i = 1:numOTpoints
    H_OT_to_EMCS_cell{1}(:,:,i) = Y*H_OT_to_OCS_cell{1}(:,:,i);
end

% plot ot frame into EMCS plot
Plot_points(H_OT_to_EMCS_cell, EMCS_plot_handle)
% break
%nice!
plotEnvironment(EMCS_plot_handle,[],Y)

end