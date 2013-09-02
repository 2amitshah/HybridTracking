%UNDER PROGRESS
%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to NDI EM and OT datasets,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT tracker.
%Calibration matrix X is calculated from OT to each EM tracker.
%%%%%%%%%%%%%
function [H_OT_to_EMT_cell, H_OT_to_EMT, errors] = calibration_OT_to_each_EMT(path, testrow_name_EMT, testrow_name_OT)
%% data read in

close all;

if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack' filesep 'CalibrationForPaper'];
end
if ~exist('testrow_name_EMT', 'var')
    testrow_name_EMT = 'EM_';
end

if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OT_';
end

% get data for hand/eye calib
[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
[H_EMT_to_EMCS_cell] = trackingdata_to_matrices(data_EMT, 'NDIQuat');

[data_OT] = read_NDI_tracking_files(path, testrow_name_OT);
[H_OT_to_OCS_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');

numPts = size(data_EMT,1);
numSen = size(data_EMT,2);

H_OT_to_EMT_cell = cell(1,numSen);
errors = H_OT_to_EMT_cell;

%% find out poses best suited for calibration
first_valid_index = 0;
for i = 1:numPts
    if ~(H_EMT_to_EMCS_cell{1}(4,4,i)==0) % if pose exists
        first_valid_index = i;
        break;
    end
end
if first_valid_index == 0
    error('something''s wrong!')
else
    pose_indices = first_valid_index;
end
for i = (first_valid_index+1):numPts
    H_temp = H_EMT_to_EMCS_cell{1}(:,:,i);
    if ~(H_temp(4,4)==0) % if pose exists
        pose_ok = true;
        for k=1:numel(pose_indices)
            H_old = H_EMT_to_EMCS_cell{1}(:,:,pose_indices(k));
            % find out rotation from old to current pose
            H_diff = H_temp / H_old;
            % get angle of rotation
            RVec_diff = rodrigues(H_diff(1:3,1:3));
            angle_rad = norm(RVec_diff);
            angle_deg = angle_rad*180/pi;
            if angle_deg < 80
                pose_ok = false;
            end
        end
        if(pose_ok)
            pose_indices = [pose_indices, i];
        end
    end
end
if numel(pose_indices) < 3
    error('too few valid poses found for calibration')
end

%% plot position data
H_EMT_to_EMCS_cell_part{1} = H_EMT_to_EMCS_cell{1}(:,:,pose_indices);
H_OT_to_OCS_cell_part{1} = H_OT_to_OCS_cell{1}(:,:,pose_indices);
EMCSfigure = Plot_frames(H_EMT_to_EMCS_cell_part);
OCSfigure = Plot_frames(H_OT_to_OCS_cell_part);

%% calibration
for j = 1:numSen

    H_EMT_to_EMCS = H_EMT_to_EMCS_cell{j};
    [X_1_Laza, err_Laza] = handEyeLaza(H_OT_to_OCS_cell{1}(:,:,pose_indices), H_EMT_to_EMCS(:,:,pose_indices));
    % we get EMT to OT here (=X_1_Laza)
    % so it needs to be inverted
    H_OT_to_EMT=inv(X_1_Laza);
    disp 'Distance according to modified algorithm:'
    disp(norm(H_OT_to_EMT(1:3,4)))

    H_OT_to_EMT_cell{j}.X = H_OT_to_EMT;
    H_OT_to_EMT_cell{j}.error = err_Laza;

    errors{j} = err_Laza;
end

%% plot OT and cylinder in EMCS frame
%plot the transformation of OT in OCS into OT in EMCS
opticalPoints_EMCS_transl = zeros(4,numPts);
H_OT_to_EMT = H_OT_to_EMT_cell{1}.X;
H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
for i=pose_indices
    temp_EMT_to_EMCS=H_EMT_to_EMCS(:,:,i);
    opticalPoints_EMCS_transl(:,i) = temp_EMT_to_EMCS * H_OT_to_EMT * [0;0;0;1];
end
% crop 4x1 vectors to 3x1
opticalPoints_EMCS=opticalPoints_EMCS_transl(1:3,:);

figure(EMCSfigure)
hold on
plot3(opticalPoints_EMCS(1,pose_indices), opticalPoints_EMCS(2,pose_indices), opticalPoints_EMCS(3,pose_indices), 'rx', 'MarkerSize', 5)
hold off

% show which OT and EMT should correlate
hold on
line([opticalPoints_EMCS(1,pose_indices); permute(H_EMT_to_EMCS(1,4,pose_indices),[1 3 2])],...
    [opticalPoints_EMCS(2,pose_indices); permute(H_EMT_to_EMCS(2,4,pose_indices),[1 3 2])],...
    [opticalPoints_EMCS(3,pose_indices); permute(H_EMT_to_EMCS(3,4,pose_indices),[1 3 2])], 'LineWidth',1,'Color', [0 0 0]);
hold off

%make the plot nicer
%plot cylinder which looks like our tool
r_cylinder = 26.5/2;
[Xcy,Ycy,Zcy] = cylinder(r_cylinder,20);

r_sphere = 4;
[Xsp, Ysp, Zsp] = sphere;
Xsp = Xsp * r_sphere;
Ysp = Ysp * r_sphere;
Zsp = Zsp * r_sphere;

H_EMT_to_EMCS_tmp = H_EMT_to_EMCS;

for i=pose_indices

    %remove offset
    Zcy_temp = Zcy-.5;
    %stretch to correct length
    Zcy_temp = Zcy_temp*100; %10cm
    
    %rearrange EMT local cordinate system, so z points along the tool and
    %not orthogonal to it
    H_EMT_to_EMCS_tmp(:,:,i)=H_EMT_to_EMCS(:,[2 3 1 4],i);
    
    Xcy_temp = H_EMT_to_EMCS_tmp(1,1,i)*Xcy + H_EMT_to_EMCS_tmp(1,2,i)*Ycy + H_EMT_to_EMCS_tmp(1,3,i)*Zcy_temp;
    Ycy_temp = H_EMT_to_EMCS_tmp(2,1,i)*Xcy + H_EMT_to_EMCS_tmp(2,2,i)*Ycy + H_EMT_to_EMCS_tmp(2,3,i)*Zcy_temp;
    Zcy_temp = H_EMT_to_EMCS_tmp(3,1,i)*Xcy + H_EMT_to_EMCS_tmp(3,2,i)*Ycy + H_EMT_to_EMCS_tmp(3,3,i)*Zcy_temp;    
       
    %shift cylinder along axes to be positioned like our tool
    %center at EMT position
    Xcy_temp = Xcy_temp + H_EMT_to_EMCS(1,4,i);
    Ycy_temp = Ycy_temp + H_EMT_to_EMCS(2,4,i);
    Zcy_temp = Zcy_temp + H_EMT_to_EMCS(3,4,i);
    %shift along radius, plus 3mm offset, since tool looks like that
    Xcy_temp = Xcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(1,2,i);
    Ycy_temp = Ycy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(2,2,i);
    Zcy_temp = Zcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(3,2,i);
    %shift along new z axis to represent position of EMT on tool correctly
    Xcy_temp = Xcy_temp + 30*H_EMT_to_EMCS_tmp(1,3,i);
    Ycy_temp = Ycy_temp + 30*H_EMT_to_EMCS_tmp(2,3,i);
    Zcy_temp = Zcy_temp + 30*H_EMT_to_EMCS_tmp(3,3,i);
    %shift EMT 7mm to the side, since EMT local coordinate system is not
    %defined in the center of the tracker disc, but at the bottom of one of the
    %screws (the right one)
    Xcy_temp = Xcy_temp - 7*H_EMT_to_EMCS_tmp(1,1,i);
    Ycy_temp = Ycy_temp - 7*H_EMT_to_EMCS_tmp(2,1,i);
    Zcy_temp = Zcy_temp - 7*H_EMT_to_EMCS_tmp(3,1,i);
    
    %plot cylinder
    hold on
    surf(Xcy_temp, Ycy_temp, Zcy_temp, 'EdgeColor', 'none', 'FaceColor', 'y', 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
    hold off
    %plot little sphere in grey to look like the OT
    hold on
    surf(Xsp+opticalPoints_EMCS(1,i), Ysp+opticalPoints_EMCS(2,i), Zsp+opticalPoints_EMCS(3,i), 'EdgeColor', 'none', 'FaceColor', [.9 .9 .9], 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
    hold off
    %plot white stick that connects OT to the tool
    hold on
    line([opticalPoints_EMCS(1,i);opticalPoints_EMCS(1,i)+20*H_EMT_to_EMCS_tmp(1,2,i)],...
        [opticalPoints_EMCS(2,i);opticalPoints_EMCS(2,i)+20*H_EMT_to_EMCS_tmp(2,2,i)],...
        [opticalPoints_EMCS(3,i);opticalPoints_EMCS(3,i)+20*H_EMT_to_EMCS_tmp(3,2,i)], 'Color', 'w', 'LineWidth', 3)
    hold off
end

% Plot lines that connect position-pairs that contributed to the
% calculation
k=0;
% goodCombinationsTemp = goodCombinations;
% for i = 1:numPts,
%     for j = i+1:numPts;
%         k=k+1;
%         if numel(goodCombinationsTemp~=0)
%         if k==goodCombinationsTemp(1)
%             goodCombinationsTemp = goodCombinationsTemp(2:end);
%             %plot line that connects used positions pairs
%             hold on
%             line([opticalPoints_EMCS(1,i);opticalPoints_EMCS(1,j)],...
%                 [opticalPoints_EMCS(2,i);opticalPoints_EMCS(2,j)],...
%                 [opticalPoints_EMCS(3,i);opticalPoints_EMCS(3,j)], 'Color', 'y', 'LineWidth', 4, 'LineStyle', '--')
%             hold off
%         end
%         end
%     end
% end


% set axes and lighting
xlabel('x')
ylabel('y')
zlabel('z')
axis image vis3d

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')

camlight('headlight')
lighting gouraud



end