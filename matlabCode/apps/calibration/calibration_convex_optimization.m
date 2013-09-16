% hand eye calibration
% AX = XB
% [R t; 0 1]
% R_A*R_X = R_X*R_B
% R_A*t_X_ + t_A = R_X*t_B + t_X 
% convex optimization formulation by orthonormal matrix
% min(x)max(i) {fi(x) = L2norm(Ci*x - di)}   for i = 1, ...., N
% where
% x = [vec(R_X) t_X]'
% Ci = [I9 - R_Ai#R_Bi 0(9x3); I3#t_Bi' I3 - R_Ai] # Knonecher product
% di = [0(9x1) t_Ai]

function [H_OT_to_EMT_cell, H_OT_to_EMT, errors] = calibration_OT_to_each_EMT(path, testrow_name_EMT, testrow_name_OT)
angle_minimum = 20;
rotation_vector_minimum = 0;
%% data read in

close all;
if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
end

if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack' filesep '3rdCalibration'];
end
if ~exist('testrow_name_EMT', 'var')
    testrow_name_EMT = 'EM_';
end

if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OT_';
end

% get data for hand/eye calib
[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
[H_EMT_to_EMCS_cell, H_EMCS_to_EMT_cell] = trackingdata_to_matrices(data_EMT, 'NDIQuat');

[data_OT] = read_NDI_tracking_files(path, testrow_name_OT);
[H_OT_to_OCS_cell, H_OCS_to_OT_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');

numPts = size(data_EMT,1);
numSen = size(data_EMT,2);

H_OT_to_EMT_cell = cell(1,numSen);
errors = H_OT_to_EMT_cell;

H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
H_OT_to_OCS = H_OT_to_OCS_cell{1};
H_OCS_to_OT=H_OCS_to_OT_cell{1};
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
        Lpose = numel(pose_indices);
        RVec_diff = cell(1,Lpose);
        angle_rad = RVec_diff;
        % check for rotation angles
        for k=1:Lpose
            H_old = H_EMT_to_EMCS_cell{1}(:,:,pose_indices(k));
            % find out rotation from old to current pose
            H_diff = H_temp / H_old;
            % get angle of rotation
            RVec_diff{k} = rodrigues(H_diff(1:3,1:3));
            angle_rad{k} = norm(RVec_diff{k});
            angle_deg = angle_rad{k}*180/pi;
            if angle_deg < angle_minimum
                pose_ok = false;
            end
        end
        % check for rotation axes
        for k=1:Lpose-1
            for l=k+1:Lpose
                sprod = dot(RVec_diff{k}/angle_rad{k}, RVec_diff{l}/angle_rad{l});
                if acos(sprod)*180/pi < rotation_vector_minimum
                    pose_ok = false;
                end
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
%% optimization
R_A = H_OT_to_OCS(1:3,1:3,pose_indices);
t_A = H_OT_to_OCS(1:3,4,pose_indices);
R_B = H_EMT_to_EMCS(1:3,1:3,pose_indices);
t_B = H_EMT_to_EMCS(1:3,4,pose_indices);
I9 = eye(9);
I3 = eye(3);
O = zeros(9,3);
for i = 1: numel(pose_indices) %numPts
    C(:,:,i) = [I9-kron(R_A(:,:,i), R_B(:,:,i)) O; kron(I3,t_B(:,:,i)') I3-R_A(:,:,i)];
    %x = [R_X(:)' t_x];
    d(:,:,i) = [zeros(9,1)' t_A(:,:,i)']';
end
fun_handle = @(x) fun(x,C,d); %norm(C*x - d);
%Initialize with H obtained from Tsai 
x_init = [0.473203514528387,0.344090055777984,-0.810975010313318,-9.49318044041395;-0.556943096633458,0.830104146444943,0.0272303721466953,-37.5868343692097;0.682563418995128,0.438781425734518,0.584446780708103,17.9659834551067;0,0,0,1;];
%x_init = [-0.854481722239389,0.0977674845919582,-0.510198495994813,-11.1435840869692;-0.371668621117342,0.571111187725496,0.731911502390492,-2.40800249893293;0.362937215555999,0.815029772591979,-0.451666965089461,-49.9117247739438;0,0,0,1;]; %H_OT_EMT_cell{1};
%x_init = [0.1 0.1 0.1 1; 0.1 0.1 0.1 1; 0.1 0.1 0.1 1; 0 0 0 1;];
R0 = x_init(1:3,1:3);
t0 = x_init(1:3,4);
x0 = [R0(:)' t0']';

options = optimset('MaxFunEvals', 10000, 'MaxIter', 10000, 'Display','iter');
[x,fval] = fminimax(fun_handle,x0 ,...
                    [],[],[],[],[],[],[],options);
               
Rx = vec2mat(x(1:9),3);
tx = x(10:12);
%H_OT_to_EMT_cell{1} = [Rx1 tx1; 0 0 0 1];
H_OT_to_EMT = [Rx tx; 0 0 0 1];

%options = optimset('MinAbsMax',5); 
%[x2,fval2] = fminsearch(fun_handle,x0) 
% Rx2 = vec2mat(x2(1:9),3);
% tx2 = x2(10:12);
% %H_OT_to_EMT_cell{1} = [Rx tx; 0 0 0 1];
% H_OT_to_EMT2 = [Rx2 tx2; 0 0 0 1];
%% Y computation
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * (H_OT_to_EMT * H_OCS_to_OT(:,:,i));
end

%% median to find Y
% all indices in which polaris system is at least above aurora system
negative_z_bool_indices = Y_all(3,4,:)<0;
median_of_z = median(Y_all(3,4,negative_z_bool_indices));
median_bool_indices = Y_all(3,4,:)<(median_of_z+2) & Y_all(3,4,:)>(median_of_z-2);

if ~any(median_bool_indices)
    error('polaris_to_aurora: heights of polaris differ more than 2mm from mean height, change tolerance or algorithm altogether')
end
Y = mean_transformation(Y_all(:,:,median_bool_indices));

%% Horns method to improve Y
pointSetOTByEMT = zeros(3,numPts);
pointSetOT = zeros(3,numPts);
for i = 1:numPts
   tmp_point_OT_by_EMT = H_EMT_to_EMCS_cell{1}(:,:,i) * H_OT_to_EMT;
   pointSetOTByEMT(:,i) = tmp_point_OT_by_EMT(1:3,4);
   tmp_point_OT = Y * H_OT_to_OCS_cell{1}(:,:,i);
   pointSetOT(:,i) = tmp_point_OT(1:3,4);
end
[T,~,ErrorStats] = absor(pointSetOT,pointSetOTByEMT);
AbsorError = ErrorStats.errlsq / sqrt(numPts);
if strcmp(verbosity,'vDebug')
    disp 'Correction of Y matrix by point-fit:'
    disp(T.M)
end
disp 'Remaining RMS translation error of Y:'
disp(AbsorError)

if strcmp(verbosity,'vDebug')
    H_OT_to_EMCS = zeros(4,4,numPts);
    for i = 1:numPts
        H_OT_to_EMCS(:,:,i) = Y * H_OT_to_OCS_cell{1}(:,:,i);
    end
    wrapper{1}=H_OT_to_EMCS;
    absorfigure = Plot_points(wrapper, [], 1); 
end

Y = T.M * Y;

%% distance between OT and EM sensor


%%
    function L_infinity_norm = fun(x,C,d)
        N=size(d,3);
        err=zeros(1,N);
        for j=1:N
            err(j)=norm(C(:,:,j)*x-d(:,:,j));
        end
        L_infinity_norm=err;
    end
end