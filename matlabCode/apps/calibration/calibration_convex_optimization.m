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

%% data read in

close all;

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
H_OT_to_OCS = H_OT_to_OCS_cell{1};
%% optimization
R_A = H_OT_to_OCS(1:3,1:3,:);
t_A = H_OT_to_OCS(1:3,4,:);
R_B = H_EMT_to_EMCS(1:3,1:3,:);
t_B = H_EMT_to_EMCS(1:3,4,:);
I9 = eye(9);
I3 = eye(3);
O = zeros(9,3);
for i = 1: numPts
    C(:,:,i) = [I9-kron(R_A(:,:,i), R_B(:,:,i)) O; kron(I3,t_B(:,:,i)') I3-R_A(:,:,i)];
    %x = [R_X(:)' t_x];
    d(:,:,i) = [zeros(9,1)' t_A(:,:,i)']';
end
fun_handle = @(x) fun(x,C,d); %norm(C*x - d);
%Initialize with H obtained from Tsai 
x_init = [-0.854481722239389,0.0977674845919582,-0.510198495994813,-11.1435840869692;-0.371668621117342,0.571111187725496,0.731911502390492,-2.40800249893293;0.362937215555999,0.815029772591979,-0.451666965089461,-49.9117247739438;0,0,0,1;]; %H_OT_EMT_cell{1};
R0 = x_init(1:3,1:3);
t0 = x_init(1:3,4);
x0 = [R0(:)' t0']';

[x,fval] = fminimax(fun_handle,x0)
Rx = vec2mat(x(1:9),3);
tx = x(10:12);
H_OT_to_EMT_cell{1} = [Rx tx; 0 0 0 1];
    function L_infinity_norm = fun(x,C,d)
        N=size(d,3);
        err=zeros(1,N);
        for j=1:N
            err(j)=norm(C(:,:,j)*x-d(:,:,j));
        end
        L_infinity_norm=max(err);
    end
end