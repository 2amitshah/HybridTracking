%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to one EMT and one OT dataset,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT.
%calibration is done for Optical to the first EM tracker.
%%%%%%%%%%%%%

function [H_OT_to_EMT, errors] = calibration_OT_to_common_EMT(path, testrow_name_EMT, testrow_name_OT)
%% data read in
% do preparation
close all;

if ~exist('path', 'var')
    path = '..\measurements\testmfrom_NDItrack';
end
testrow_name_EMT = 'hybridEMT';
% testrow_name_OT = 'hybridOT';

% get data for hand/eye calib
[H_EMT_to_EMCS, H_EMCS_to_EMT] = common_EMT_frame(path, testrow_name_EMT);

[data_OT] = read_NDI_tracking_files(path, testrow_name_OT);
[H_OT_to_OCS_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');

%% plot position data
wrapper{1}=H_EMT_to_EMCS;
EMCSfigure = Plot_frames(wrapper);


OCSfigure = Plot_frames(H_OT_to_OCS_cell);


%% calibration

[Hcam2marker_, err] = TSAIleastSquareCalibration(H_OT_to_OCS_cell{1}, H_EMCS_to_EMT)
% we get EMT to OT here (~Hcam2marker_)
% so lets turn it around
X_1 = inv(Hcam2marker_);
disp 'Distance according to TSAI algorithm:'
disp(norm(X_1(1:3,4)))

[X_1_Laza, err_Laza, goodCombinations] = handEyeLaza_goodcombinations(H_OT_to_OCS_cell{1}, H_EMT_to_EMCS)
% we get EMT to OT here (~X_1_Laza)
% so lets turn it around
H_OT_to_EMT=inv(X_1_Laza);
disp 'Distance according to modified algorithm:'
disp(norm(H_OT_to_EMT(1:3,4)))

%% plot OT inside the EM-CS
numPts = size(H_EMT_to_EMCS,3);
%plot the transformation of OT in OCS into OT in EMCS
opticalPoints_EMCS_transl = zeros(4,numPts);

for i=1:numPts
    temp_EMT_to_EMCS=H_EMT_to_EMCS(:,:,i);
    opticalPoints_EMCS_transl(:,i) = temp_EMT_to_EMCS * (H_OT_to_EMT * [0;0;0;1]);
end

opticalPoints_EMCS=opticalPoints_EMCS_transl(1:3,:);

figure(EMCSfigure)
hold on
plot3(opticalPoints_EMCS(1,:), opticalPoints_EMCS(2,:), opticalPoints_EMCS(3,:), 'rx', 'MarkerSize', 5)
hold off

% show which OT and EMT should correlate
hold on
line([opticalPoints_EMCS(1,:); permute(H_EMT_to_EMCS(1,4,:),[1 3 2])],...
    [opticalPoints_EMCS(2,:); permute(H_EMT_to_EMCS(2,4,:),[1 3 2])],...
    [opticalPoints_EMCS(3,:); permute(H_EMT_to_EMCS(3,4,:),[1 3 2])], 'LineWidth',1,'Color', [0 0 0]);
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

for i=1:numPts

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
goodCombinationsTemp = goodCombinations;
for i = 1:numPts,
    for j = i+1:numPts;
        k=k+1;
        if numel(goodCombinationsTemp~=0)
        if k==goodCombinationsTemp(1)
            goodCombinationsTemp = goodCombinationsTemp(2:end);
            %plot line that connects used positions pairs
            hold on
            line([opticalPoints_EMCS(1,i);opticalPoints_EMCS(1,j)],...
                [opticalPoints_EMCS(2,i);opticalPoints_EMCS(2,j)],...
                [opticalPoints_EMCS(3,i);opticalPoints_EMCS(3,j)], 'Color', 'y', 'LineWidth', 4, 'LineStyle', '--')
            hold off
        end
        end
    end
end


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