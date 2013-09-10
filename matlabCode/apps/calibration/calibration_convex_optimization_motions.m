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
[H_OT_to_OCS_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');

numPts = size(data_EMT,1);
numSen = size(data_EMT,2);

H_OT_to_EMT_cell = cell(1,numSen);
errors = H_OT_to_EMT_cell;

H_EMT_to_EMCS = H_EMT_to_EMCS_cell{1};
H_EMCS_to_EMT = H_EMCS_to_EMT_cell{1};
H_OT_to_OCS = H_OT_to_OCS_cell{1};
%% build all possible motion pairs
k=0;
numMotions = (numPts*(numPts-1))/2; % Gaussian sum
R_A = cell(1,numMotions);
R_B = R_A;
t_A = R_A;
t_B = R_A;
C = R_A;
d = R_A;
for i = 1:numPts,
    for j = i+1:numPts;
		H_OT_ij = (H_OT_to_OCS(:,:,j))\H_OT_to_OCS(:,:,i);    % Transformation from i-th to j-th OT pose
% 		Pgij = 2*rot2quat(H_OT_ij);            % ... and the corresponding quaternion
      
		H_EM_ij = H_EMCS_to_EMT(:,:,j)/(H_EMCS_to_EMT(:,:,i));    % Transformation from i-th to j-th EM pose
% 		Pcij = 2*rot2quat(H_EM_ij);            % ... and the corresponding quaternion

        k = k+1;                            % Form linear system of equations
        R_A{k}=H_OT_ij(1:3,1:3);
        R_B{k}=H_EM_ij(1:3,1:3);
        t_A{k}=H_OT_ij(1:3,4);
        t_B{k}=H_EM_ij(1:3,4);
        
        C{k} = [ [eye(9)-kron(R_A{k},R_B{k}) zeros(9,3)]; [kron(eye(3),(t_B{k})') eye(3)-R_A{k}]];
        d{k} = [zeros(9,1); t_A{k}];
    end;
end;
%% concatenate matrices for SeDuMi
% sedumi(A,b)
Cs=zeros(12*k, 12);
ds=zeros(12*k,1);
for i=1:k
Cs(12*(i-1)+1:12*i,:)=C{i};
ds(12*(i-1)+1:12*i,:)=d{i};
end
[x,y,info]=sedumi(Cs,ds,0);
disp(x)
return
%% optimization

obj_fcn_handle = @(x) convex_obj_fcn(x,C,d);

x0 = zeros(12,1);
options = optimset('MaxFunEvals', 5000, 'MaxIter', 10000);
[x,delta] = fminsearch(obj_fcn_handle, x0, options);

%% iterate while excluding motion pairs from the solution
N_new_latest=0;
whilecounter=0;
while delta>10
    N=numel(C);
    err=zeros(1,N);
    for i=1:N
        err(i)=norm(C{i}*x-d{i});
    end
    bad_indices = err>max(err)-1;
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
        [x,delta] = fminsearch(obj_fcn_handle, x, options);
        whilecounter = whilecounter+1;
    else
        break;
    end
end
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
end