% hand eye calibration
% AX = XB
% [R t; 0 1]
% R_A*R_X = R_X*R_B
% R_A*t_X_ + t_A = R_X*t_B + t_X 
% convex optimization formulation by orthonormal matrix
% min(x)max(i) {fi(x) = L2norm(Ci*x - di)}   for i = 1, ...., N
% where
% x = [vec(R_X) t_X]'
% Ci = [I9 - R_Ai#R_Bi 0(9x3); I3#t_Bi' I3 - R_Ai] # Kronecker product, matlab function: kron(A,B)
% di = [0(9x1) t_Ai]

function [H_OT_to_EMT_cell, H_OT_to_EMT, errors] = calibration_convex_optimization_motions(path, testrow_name_EMT, testrow_name_OT)

%% data read in

close all;clc;

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
H_EMCS_to_EMT = H_EMCS_to_EMT_cell{1};
H_OT_to_OCS = H_OT_to_OCS_cell{1};
H_OCS_to_OT = H_OCS_to_OT_cell{1};
%% build all possible motion pairs
k=0;
numMotions = (numPts*(numPts-1))/2; % Gaussian sum
R_A = cell(1,numMotions);
R_B = R_A;
t_A = R_A;
t_B = R_A;
C = R_A;
d = R_A;
for i = 1:numPts
    for j = i+1:numPts
		%H_OT_ij = (H_OT_to_OCS(:,:,j))\H_OT_to_OCS(:,:,i);    % Transformation from i-th to j-th OT pose
% 		Pgij = 2*rot2quat(H_OT_ij);            % ... and the corresponding quaternion
      
		%H_EM_ij = H_EMCS_to_EMT(:,:,j)/(H_EMCS_to_EMT(:,:,i));    % Transformation from i-th to j-th EM pose
% 		Pcij = 2*rot2quat(H_EM_ij);            % ... and the corresponding quaternion

        k = k+1;                            % Form linear system of equations
        R_A{k}=H_OT_to_OCS(1:3,1:3,j)'*H_OT_to_OCS(1:3,1:3,i); %H_OT_ij(1:3,1:3);
        R_B{k}=H_EMT_to_EMCS(1:3,1:3,j)'*(H_EMT_to_EMCS(1:3,1:3,i)); %H_EM_ij(1:3,1:3);
        t_A{k}=H_OT_to_OCS(1:3,1:3,j)'*(H_OT_to_OCS(1:3,4,i)-H_OT_to_OCS(1:3,4,j)); %H_OT_ij(1:3,4);
        t_B{k}=H_EMT_to_EMCS(1:3,1:3,j)'*(H_EMT_to_EMCS(1:3,4,i)-H_EMT_to_EMCS(1:3,4,j)); %H_EM_ij(1:3,4);
        
        C{k} = [ [eye(9)-kron(R_A{k},R_B{k}) zeros(9,3)]; [kron(eye(3),(t_B{k})') eye(3)-R_A{k}]];
        d{k} = [zeros(9,1); t_A{k}];
    end
end
%% concatenate matrices for SeDuMi
% sedumi(A,b)
% Cs=zeros(12*k, 12);
% ds=zeros(12*k,1);
% for i=1:k
% Cs(12*(i-1)+1:12*i,:)=C{i};
% ds(12*(i-1)+1:12*i,:)=d{i};
% end
% [x,y,info]=sedumi(Cs,ds,0);
% disp(x)
% return
%% optimization

obj_fcn_handle = @(x) func(x,C,d);

x0 = zeros(12,1);
x_init = [0.473203514528387,0.344090055777984,-0.810975010313318,-9.49318044041395;-0.556943096633458,0.830104146444943,0.0272303721466953,-37.5868343692097;0.682563418995128,0.438781425734518,0.584446780708103,17.9659834551067;0,0,0,1;];
R0 = x_init(1:3,1:3);
t0 = x_init(1:3,4);
%x0 = [R0(:)' t0']';
options = optimset('MaxFunEvals', 10000, 'MaxIter', 10000, 'Display','iter');
[x,delta] = fminimax(obj_fcn_handle, x0, ...
    [], [], [], [], [], [], [], options);
% options = optimset('MaxFunEvals', 5000, 'MaxIter', 10000);
% [x1,delta1] = fminsearch(obj_fcn_handle, x0, options);

Rx = vec2mat(x(1:9),3);
tx = x(10:12);
%H_OT_to_EMT_cell{1} = [Rx1 tx1; 0 0 0 1];
H_OT_to_EMT = [Rx tx; 0 0 0 1]


 %% iterate while excluding motion pairs from the solution
N_new_latest=0;
whilecounter=0;
error = 30;
%del = delta;
flag = 1;
while flag
    
    N=numel(C);
    bad_indices=zeros(1,N);
    for i=1:N
        if(norm(C{i}*x-d{i})>error)
           bad_indices(i) = 1; 
        end
    end
    
    N_new = sum(~bad_indices);
    if N_new > 2
        N_new_latest = N_new;
        good_indices = find(~bad_indices);
        C_temp = cell(1,N_new);
        d_temp = cell(1,N_new);
        for i=1:N_new
            C_temp{i}=C{good_indices(i)};
            d_temp{i} = d{good_indices(i)};
            
        end
        C=C_temp;
        d=d_temp;
        
        obj_fcn_handle = @(x) convex_obj_fcn(x,C,d);
        
        if (delta<error)
            flag = 0;
        end
       %options = optimset('MaxFunEvals', 10000, 'MaxIter', 10000, 'Display','iter');
[x,delta] = fminimax(obj_fcn_handle, x0, ...
    [], [], [], [], [], [], [], options);
        whilecounter = whilecounter+1;
        
    else
        break;
    end
end
Rx = vec2mat(x(1:9),3);
tx = x(10:12);
%H_OT_to_EMT_cell{1} = [Rx1 tx1; 0 0 0 1];
H_OT_to_EMT1 = [Rx tx; 0 0 0 1]

disp('Number of "reducing the poses"-loops')
disp(whilecounter)

disp 'x:'
disp(x)
disp 'Maximum residual:'
disp(delta)
disp 'Used number of motion pairs'
disp(N_new_latest)
disp 'break while because of N?'
disp(N_new)

%% Y computation
for i = 1:numPts
    Y_all(:,:,i) = H_EMT_to_EMCS(:,:,i) * (H_OT_to_EMT1 * H_OCS_to_OT(:,:,i));
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
%plot the transformation of EM in OCS
 %ticalPoints_EMCS_transl = zeros(4,numPts);

for i=1:numPts
    opticalPoints_EMCS(:,i) = H_OT_to_OCS(1:3,4,i);
   %emPointsFirstSensor(:,i) = H_EMT_to_EMCS(1:3,4,i);
    temp_EMT_to_EMCS=H_EMT_to_EMCS(:,:,i);
    emPointsFirstSensor(:,i) = temp_EMT_to_EMCS * (H_OT_to_EMT1 * [0;0;0;1]);
end

diff=opticalPoints_EMCS-emPointsFirstSensor;
 disp 'Distance from OT to EMT in all positions:'
 disp(norm(diff(:,1)));
%%
function L_infinity_norm = func(x,C,d)
    N=numel(C);
    err=zeros(1,N);
    for i=1:N
        err(i)=norm(C{i}*x-d{i});
    end
    L_infinity_norm=err;
end
end