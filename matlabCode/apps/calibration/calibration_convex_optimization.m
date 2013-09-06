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
H_OT_to_OCS = H_OT_to_OCS_cell{1};
%% optimization
R_A = H_OT_to_OCS(1:3,1:3);
t_A = H_OT_to_OCS(1:3,4);
R_B = H_EMT_to_EMCS(1:3,1:3);
t_B = H_EMT_to_EMCS(1:3,4);
I9 = eye(9);
I3 = eye(3);
O = zeros(9,3);
C = [I9-kron(R_A, R_B) O; kron(I3,t_B') I3-R_A];
%x = [R_X(:)' t_x];
d = [zeros(9,1)' t_A']';
fun = @(x) C*x - d;
x0 = ones(9+3,1);
H_OT_to_EMT = lsqnonlin(fun, x0);
% while delta <= e
%     for i = 1 : numPts
%         if (norm(C(i)*x - d(i) > e))
%             %remove ith motion from C and d;
%         end
%     end
% end
end