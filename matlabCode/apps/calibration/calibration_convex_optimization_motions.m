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
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack' filesep 'trus_recording'];
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
        t_A{k}=H_OT_ij(4,1:3);
        t_B{k}=H_EM_ij(4,1:3);
        
        C{k} = [ [eye(9)-kron(R_A{k},R_B{k}) zeros(9,3)]; [kron(eye(3),t_B{k}') eye(3)-R_A{k}]];
        d{k} = [zeros(9,1); t_A{k}];
    end;
end;

%% optimization
% R_A = H_OT_to_OCS(1:3,1:3);
% t_A = H_OT_to_OCS(1:3,4);
% R_B = H_EM_to_EMCS(1:3,1:3);
% t_B = H_EM_to_EMCS(1:3,4);
% I9 = eye(9);
% I3 = eye(3);
% O = zeros(9,3);
% C = [I9-kron(R_A, R_B) O; kron(I3,t_B') I3-R_A];
% %x = [R_X(:)' t_x];
% d = [zeros(9,1) t_A]';

obj_fcn_handle = @(x)convex_obj_fcn(x,C,d);
%fun = C*x - d;
x0 = ones(12,1);
% x = lsqnonlin(C*x - d, x0)
[x,delta] = fminsearch(obj_fcn_handle, x0, 'Display', 'iter');

%% iterate while excluding motion pairs from the solution
while delta>10
    N=numel(C);
    err=zeros(1,N);
    for i=1:N
        err(i)=norm(C{i}*x-d{i});
    end
    bad_indices = err>10;
    N_new = sum(~bad_indices);
    C_temp = cell(1,N_new);
    d_temp = cell(1,N_new);
    for i=1:N_new
        C_temp{i}=C{i};
        d_temp = d{i});
    end
end

% while delta <= e
%     for i = 1 : numPts
%         if (norm(C(i)*x - d(i) > e))
%             %remove ith motion from C and d;
%         end
%     end
% end
end