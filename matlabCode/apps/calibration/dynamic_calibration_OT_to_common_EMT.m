% dynamic_calibration_OT_to_common_EMT
%
% Author: Felix Achilles, July 2013

function [H_OT_to_EMT, errors] = dynamic_calibration_OT_to_common_EMT(path, testrow_name_EMT, testrow_name_OT, verbosity)
%% data read in
% do preparation

if ~exist('verbosity', 'var')
    verbosity = 'vRelease';
end

filenames_struct = path;
if isstruct(filenames_struct)
    testrow_name_EMT = filenames_struct.EMfiles;
    testrow_name_OT = filenames_struct.OTfiles;
    path = filenames_struct.folder;
end

if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];
end
if ~exist('testrow_name_EMT', 'var')
    testrow_name_EMT = 'hybridEMT';
end

if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'hybridOT';
end

% get data for hand/eye calib
[dataOT, dataEM] = read_Direct_OpticalAndAscension(filenames_struct);

[interval]=obtain_boundaries_for_interpolation(dataOT, dataEM);

H_OT_to_OCS_interp = frame_interpolation(dataOT,interval,10,'cpp');
numSens = size(dataEM, 2);
H_EMT_to_EMCS_cell_interp = cell(1,numSens);
for j=1:numSens
H_EMT_to_EMCS_cell_interp{j} = frame_interpolation(dataEM(:,j),interval,10,'ndi');
end
% [H_commonEMT_to_EMCS] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell_interp, verbosity, 'ndi');
H_commonEMT_to_EMCS = H_EMT_to_EMCS_cell_interp{1};


%% plot position data
if strcmp(verbosity, 'vDebug')
H_commonEMT_to_EMCS_cell{1}=H_commonEMT_to_EMCS;
EMCSfigure = Plot_points(H_commonEMT_to_EMCS_cell);
title('EMT point ins EMCS')
H_OT_to_OCS_cell{1} = H_OT_to_OCS_interp;
OCSfigure = Plot_points(H_OT_to_OCS_cell);
title('OT point ins OCS')
end


%% calibration

[X_1_Laza, err_Laza, goodCombinations] = handEyeLaza_goodcombinations(H_OT_to_OCS_interp, H_commonEMT_to_EMCS)
% we get EMT to OT here (~X_1_Laza)
% so lets turn it around
H_OT_to_EMT=inv(X_1_Laza);
disp 'Distance according to modified Tsai/Lenz algorithm:'
disp(norm(H_OT_to_EMT(1:3,4)))

errors = err_Laza;
%% plot OT inside the EM-CS
if strcmp(verbosity, 'vDebug')
numPts = size(H_commonEMT_to_EMCS,3);
%plot the transformation of OT in OCS into OT in EMCS
opticalPoints_EMCS_transl = zeros(4,numPts);

for i=1:numPts
    temp_EMT_to_EMCS=H_commonEMT_to_EMCS(:,:,i);
    opticalPoints_EMCS_transl(:,i) = temp_EMT_to_EMCS * H_OT_to_EMT * [0;0;0;1];
end
% crop 4x1 vectors to 3x1
opticalPoints_EMCS=opticalPoints_EMCS_transl(1:3,:);


figure(EMCSfigure)
hold on
plot3(opticalPoints_EMCS(1,:), opticalPoints_EMCS(2,:), opticalPoints_EMCS(3,:), 'rx', 'MarkerSize', 5)
hold off
% show which OT and EMT should correlate
hold on
line([opticalPoints_EMCS(1,:); permute(H_commonEMT_to_EMCS(1,4,:),[1 3 2])],...
    [opticalPoints_EMCS(2,:); permute(H_commonEMT_to_EMCS(2,4,:),[1 3 2])],...
    [opticalPoints_EMCS(3,:); permute(H_commonEMT_to_EMCS(3,4,:),[1 3 2])], 'LineWidth',1,'Color', [0 0 0]);
hold off
end

if strcmp(verbosity, 'vEyecandy')
%make the plot nicer
%plot cylinder which looks like our tool
r_cylinder = 26.5/2;
[Xcy,Ycy,Zcy] = cylinder(r_cylinder,20);

r_sphere = 4;
[Xsp, Ysp, Zsp] = sphere;
Xsp = Xsp * r_sphere;
Ysp = Ysp * r_sphere;
Zsp = Zsp * r_sphere;

H_EMT_to_EMCS_tmp = H_commonEMT_to_EMCS;

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
end

if strcmp(verbosity, 'vEyecandy')
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
end

% set axes and lighting
xlabel('x')
ylabel('y')
zlabel('z')
axis image vis3d

set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
set(gca,'Color','none')

if strcmp(verbosity, 'vEyecandy')
camlight('headlight')
lighting gouraud
end


end