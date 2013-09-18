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
    path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack' filesep '2ndCalibration'];
end
if ~exist('testrow_name_EMT', 'var')
    testrow_name_EMT = 'EM_';
end

if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OT_';
end

if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
end

% get data for hand/eye calib
[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
[~, H_EMCS_to_EMT_cell] = trackingdata_to_matrices(data_EMT, 'NDIQuat');

[data_OT] = read_NDI_tracking_files(path, testrow_name_OT);
[H_OT_to_OCS_cell] = trackingdata_to_matrices(data_OT, 'NDIQuat');

numPts = size(data_EMT,1);
numSen = size(data_EMT,2);

H_OT_to_EMT_cell = cell(1,numSen);
errors = H_OT_to_EMT_cell;

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
% % sedumi(A,b)
% Cs=zeros(12*k, 12);
% ds=zeros(12*k,1);
% for i=1:k
% Cs(12*(i-1)+1:12*i,:)=C{i};
% ds(12*(i-1)+1:12*i,:)=d{i};
% end
% [x,y,info]=sedumi(Cs,ds,0);
% disp(x)
% return

%% initial optimization

obj_fcn_handle = @(x) convex_obj_fcn(x,C,d);

x0 = zeros(12,1); % zeros are no good starting point, take latest tsai result
load('H_OT_to_EMT')
H_OT_to_EMT = inv(H_OT_to_EMT);
R_OT_to_EMT = H_OT_to_EMT(1:3,1:3)';
x0(1:9) = R_OT_to_EMT(1:9);
x0(10:12) = H_OT_to_EMT(1:3,4);
options = optimset('MaxFunEvals', 5000, 'MaxIter', 10000);

fun_handle = @(x) fun(x0,C,d); %norm(C*x - d);
lowerBound = [-ones(9,1);-inf;-inf;-inf];
upperBound = [ones(9,1);inf;inf;inf];

% fminimax does also not work well if no good starting point is supplied
[x,delta] = fminimax(fun_handle,x0,[],[],[],[],lowerBound,upperBound,@rotationconstraint_fcn,options);

% -fminsearch did not perform so well since it is totally unconstrained
% --(rotation matrix unitarity property was not taken into account)
% [x,delta] = fminsearch(obj_fcn_handle, x0, options);

if strcmp(verbosity, 'vDebug')
    ResidualErrorFig = figure;
    plot(delta)
    set(gca,'NextPlot','add')
end

%% iterate while excluding motion pairs from the solution
N_new_latest=0;
whilecounter=0;
delta=max(delta);
while delta>2
    N=numel(C);
    err=zeros(1,N);
    for i=1:N
        err(i)=norm(C{i}*x-d{i});
    end
    bad_indices = err>max(err)-1000*eps;
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

        fun_handle = @(x) fun(x,C,d); %norm(C*x - d);
        [x,delta] = fminimax(fun_handle,x,[],[],[],[],lowerBound,upperBound,@rotationconstraint_fcn,options);
        if strcmp(verbosity, 'vDebug')
            plot(gca,1:numel(delta),delta)
        end
        delta=max(delta);
%         obj_fcn_handle = @(x) convex_obj_fcn(x,C,d);
%         [x,delta] = fminsearch(obj_fcn_handle, x, options);
        whilecounter = whilecounter+1;
    else
        break;
    end
end
disp('Number of "reducing the poses"-loops')
disp(whilecounter)

disp 'x:'
disp(x)
disp 'All residuals:'
tmp =feval(fun_handle,x);
disp(tmp')
disp 'Maximum residual:'
disp(delta)
disp 'Used number of motion pairs'
disp(N_new_latest)
disp 'break while because of N?'
disp(N_new)

R_OT_to_EMT = reshape(x(1:9),3,3);
R_OT_to_EMT = R_OT_to_EMT';

H_OT_to_EMT = [[R_OT_to_EMT x(end-2:end)];[0 0 0 1]];
H_OT_to_EMT = inv(H_OT_to_EMT);
end

function err = fun(x,C,d)
    N=numel(C);
    err=zeros(1,N);
    for i=1:N
        err(i)=norm(C{i}*x-d{i});
    end
end

function [C, Ceq] = rotationconstraint_fcn(X)
C=-1;
R=reshape(X(1:9),3,3);
Ceq=R'*R-eye(3);
Ceq=Ceq(:);
end