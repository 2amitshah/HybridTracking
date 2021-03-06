% this function is for combining the data of the two sensors. Tthis means that
% the data of OT and EM is put into a Kalman filter as soon as it is
% available. Depending on the system from which the data is coming, the
% R-matrix is set accordingly.

function datafiltered = common_OT_EM_kalman_direct_data_V2(path, testrow_name_EM, testrow_name_OT, H_OT_to_EMT, kalmanfrequencyHz, verbosity)

if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
end
if ~exist('kalmanfrequencyHz','var')
    kalmanfrequencyHz = 4000;
end
if ~exist('path', 'var')
    pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
    path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
end
if ~exist('testrow_name_EM', 'var')
    testrow_name_EM = 'EMTrackingcont_1';
end
if ~exist('testrow_name_OT', 'var')
    testrow_name_OT = 'OpticalTrackingcont_1';
end

% get data (without any interpolation
[data_OT_tmp, data_EMT_tmp, errorTimeStampsOT, errorTimeStampsEM] = read_TrackingFusion_files(path, testrow_name_OT, testrow_name_EM);
%todo: compute the transformation between the different sensors of EM..so
%far: delete the second sensor :)

%(using static points) and transfer all the points to the position of
%sensor 1 --> result: cell with only one column
data_EM_tmp2(1:size(data_EMT_tmp,1),1) = data_EMT_tmp(1:size(data_EMT_tmp,1),1);
idx = 1;
deleteEMmin = 50;
deleteEMmax = 70;
deleteOTmin = 200;
deleteOTmax = 210;


for i = 1:size(data_EM_tmp2,1)
    if(data_EM_tmp2{i}.valid && (i <deleteEMmin || i > deleteEMmax) && (i<size(data_EM_tmp2,1)-70  || i>size(data_EM_tmp2,1)-50))
        data_EM{idx,1} = data_EM_tmp2{i};
        %data_EM{idx,1}.TimeStamp = data_EM{idx,1}.TimeStamp + 2*10^7;
        idx = idx+1;
    end
end
idx = 1;

for i = 1:size(data_OT_tmp,1)
    if(data_OT_tmp{i}.valid && (i < deleteOTmin || i > deleteOTmax) && (i<size(data_OT_tmp,1)-80 || i>size(data_OT_tmp,1)-60))
        data_OT{idx,1} = data_OT_tmp{i};
        idx = idx + 1;
    end
end


%% determine earliest and latest common timestamp
interval = obtain_boundaries_for_interpolation_earliest_latest(data_OT, data_EM);
%startTime = interval(1);
startTime = data_OT{4}.TimeStamp;
endTime = interval(2);

% get Y, equal to EMCS_to_OCS
load(which('H_OT_to_EMT.mat'));
[Y,~] = polaris_to_aurora_absor(path, H_OT_to_EMT,'cpp','static','vRelease');

%% compute EMT_by_OT data
% Relevant matrix for computing transformations
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};

%% calculate where EM tracker should be
numPtsOT = size(data_OT,1);
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPtsOT);

data_EM_by_OT = cell(numPtsOT,1);

for i = 1:numPtsOT
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT; %data_EM_common_by_OT    
    if(data_OT{i}.valid == 1) % && data_EM_common{i}.valid == 1) %DEBUG: here we limit data_EM_common_by_OT{i}.valid to only be valid when OT AND EMT at that time are valid.  
        data_EM_by_OT{i}.TimeStamp = data_OT{i}.TimeStamp;      
        data_EM_by_OT{i}.position = transpose(H_EMT_to_EMCS_by_OT(1:3,4,i));
        data_EM_by_OT{i}.orientation = transpose(rot2quat(H_EMT_to_EMCS_by_OT(1:3, 1:3, i)));   
        data_EM_by_OT{i}.valid = data_OT{i}.valid;
    else
        data_EM_by_OT{i}.valid = 0;
    end
end

%% initialize matrices and vectors
% initial state vector init_x, start with timestep 3 (in order to have the first and second
% derivative)
timestep23 = (data_EM_by_OT{3}.TimeStamp - data_EM_by_OT{2}.TimeStamp)/10^9;
timestep12 = (data_EM_by_OT{2}.TimeStamp - data_EM_by_OT{1}.TimeStamp)/10^9;
timestep13half = (data_EM_by_OT{3}.TimeStamp - data_EM_by_OT{1}.TimeStamp)/(2*10^9);
x_dot = (data_EM_by_OT{3}.position(1) - data_EM_by_OT{2}.position(1))/timestep23;
y_dot = (data_EM_by_OT{3}.position(2) - data_EM_by_OT{2}.position(2))/timestep23;
z_dot = (data_EM_by_OT{3}.position(3) - data_EM_by_OT{2}.position(3))/timestep23;
x_2dot = (data_EM_by_OT{3}.position(1) - 2*(data_EM_by_OT{2}.position(1)) + data_EM_by_OT{1}.position(1)) / (timestep13half^2);
y_2dot = (data_EM_by_OT{3}.position(2) - 2*(data_EM_by_OT{2}.position(2)) + data_EM_by_OT{1}.position(2)) / (timestep13half^2);
z_2dot = (data_EM_by_OT{3}.position(3) - 2*(data_EM_by_OT{2}.position(3)) + data_EM_by_OT{1}.position(3)) / (timestep13half^2);
%x_2dot = (x_dot - ((data_EM_by_OT{2}.position(1) - data_EM_by_OT{1}.position(1))/timestep12)) / timestep13half;
%y_2dot = (y_dot - ((data_EM_by_OT{2}.position(2) - data_EM_by_OT{1}.position(2))/timestep12)) / timestep13half;
%z_2dot = (z_dot - ((data_EM_by_OT{2}.position(3) - data_EM_by_OT{1}.position(3))/timestep12)) / timestep13half;

initx = [data_EM_by_OT{3}.position(1); data_EM_by_OT{3}.position(2); data_EM_by_OT{3}.position(3); x_dot; y_dot; z_dot; x_2dot; y_2dot; z_2dot ] 
x = initx;

statesize = 9;
observationsize = 6;
% state transition matrix A
% A = eye(statesize);
% for i = 1:6
%     A(i,i+3) = timestep_in_s;
% end
% for i = 1:3
%     A(i,i+6) = .5 * timestep_in_s^2;
% end
% A

H = zeros(observationsize,statesize);
H(1:observationsize, 1:observationsize) = eye(observationsize,observationsize);
H

%initial state covariance (P)
initV = .25*eye(statesize);
P = initV;
% process noise covariance matrix Q
Q = .25*eye(statesize);
Q(4:9,4:9)=50*eye(6);
% measurement noise covariance matrix R
R_OT = .02*eye(observationsize); %the higher the value, the less the measurement is trusted
R_OT(4:6,4:6)=100*eye(3);
R_EM = .04*eye(observationsize);
R_EM(4:6,4:6)=100*eye(3);


%% take whatever is available (OT or EM) and feed it into the Kalman filter, use prediction only when no data is coming in for a long time
t = startTime;
indexOT = 3;
indexEM = 0;
while(startTime > data_EM{indexEM+1}.TimeStamp)
    indexEM = indexEM + 1;
end

dataind = 1;
dataraw = 1;
numPtsEM = size(data_EM,1);
timestepmean = timestep13half * 10^9;
timestepcounter = 2;
timestepfactor = 3;
while(t < endTime && (indexOT < numPtsOT) && (indexEM < numPtsEM))  
    lastTimeStep = t;
    %get the index of the measurement with the next bigger timestep in
    %order to find the latest measurement
    useOT = 0;
    useEM = 0;
    newTS_OT = data_EM_by_OT{indexOT+1}.TimeStamp;
    newTS_EM = data_EM{indexEM+1}.TimeStamp;
    if(newTS_OT < newTS_EM)
        indexOT = indexOT + 1;
        useOT = 1;
        t = newTS_OT;
    else
        indexEM = indexEM + 1;
        useEM = 1;
        t = newTS_EM;
    end
    if (t-lastTimeStep > (timestepfactor*timestepmean))
        if(useEM)
            indexEM = indexEM - 1;
        end
        if(useOT)
            indexOT = indexOT - 1;
        end
        
        useEM = 0;
        useOT = 0;
        t = lastTimeStep + timestepfactor*timestepmean;
    end

    if(useOT)
        %prediction and correction with OT measurement untill its timestamp
        currentTimestep = (lastTimeStep - newTS_OT)/(10^9);
        A = eye(statesize,statesize);
        for i = 1:6
            A(i,i+3) = currentTimestep;
        end
        for i = 1:3
            A(i,i+6) = .5 * currentTimestep^2;
        end
        
        %% time update (prediction)
        x_minus = A * x;
        P_minus = A * P * transpose(A) + Q; 
        
        %% measurement update (correction)
        K = P_minus * H' * ((H * P_minus * H' + R_OT)^-1); %Kalman gain
        
        
        if(dataind > 3)
            ts = (data{dataind-1}.TimeStamp - data{dataind-2}.TimeStamp)/10^9;
            ts13mean = (data{dataind-1}.TimeStamp - data{dataind-3}.TimeStamp) / (2*10^9);
            datax_dot = (data{dataind-1}.position(1) - data{dataind-2}.position(1))/ts;
            datay_dot = (data{dataind-1}.position(2) - data{dataind-2}.position(2))/ts;
            dataz_dot = (data{dataind-1}.position(3) - data{dataind-2}.position(3))/ts;
            
            ts_minusone = (data{dataind-2}.TimeStamp - data{dataind-3}.TimeStamp)/10^9;
            datax_dot_minusone = (data{dataind-2}.position(1) - data{dataind-3}.position(1))/ts_minusone;
            datay_dot_minusone = (data{dataind-2}.position(2) - data{dataind-3}.position(2))/ts_minusone;
            dataz_dot_minusone = (data{dataind-2}.position(3) - data{dataind-3}.position(3))/ts_minusone;

                datax_2dot = (data{dataind-1}.position(1) - 2*(data{dataind-2}.position(1)) + data{dataind-3}.position(1)) / (ts13mean^2);
                datay_2dot = (data{dataind-1}.position(2) - 2*(data{dataind-2}.position(2)) + data{dataind-3}.position(2)) / (ts13mean^2);
                dataz_2dot = (data{dataind-1}.position(3) - 2*(data{dataind-2}.position(3)) + data{dataind-3}.position(3)) / (ts13mean^2);

%             datax_2dot = (datax_dot - datax_dot_minusone) / (ts13mean);
%             datay_2dot = (datay_dot - datay_dot_minusone) / (ts13mean);
%             dataz_2dot = (dataz_dot - dataz_dot_minusone) / (ts13mean);
        else
            datax_dot = x_dot;
            datay_dot = y_dot;
            dataz_dot = z_dot;
            datax_2dot = x_2dot;
            datay_2dot = y_2dot;
            dataz_2dot = z_2dot;
        end
        
        z = [ data_EM_by_OT{indexOT}.position(1); data_EM_by_OT{indexOT}.position(2); data_EM_by_OT{indexOT}.position(3); datax_dot; datay_dot; dataz_dot];%; datax_2dot;datay_2dot;dataz_2dot]; %measurement      
        x = x_minus + K * (z - (H * x_minus));
        P = (eye(statesize) - K * H ) * P_minus;
        
        rawdata{dataraw,1} = data_EM_by_OT{indexOT};
        dataraw = dataraw + 1;
        data{dataind,1}.position = x(1:3)';
        data{dataind,1}.TimeStamp = t;  
        dataind = dataind + 1;
        
        
    else
        if(useEM)
            currentTimestep = (lastTimeStep - newTS_EM)/(10^9);
            A = eye(statesize,statesize);
            for i = 1:6
                A(i,i+3) = currentTimestep;
            end
            for i = 1:3
                A(i,i+6) = .5 * currentTimestep^2;
            end

            %% time update (prediction)
            x_minus = A * x;
            P_minus = A * P * transpose(A) + Q; 

            %% measurement update (correction)
            K = P_minus * H' * ((H * P_minus * H' + R_EM)^-1); %Kalman gain
            
            if(dataind > 3)
                ts = (data{dataind-1}.TimeStamp - data{dataind-2}.TimeStamp)/10^9;
                ts13mean = (data{dataind-1}.TimeStamp - data{dataind-3}.TimeStamp) / (2*10^9);
                datax_dot = (data{dataind-1}.position(1) - data{dataind-2}.position(1))/ts;
                datay_dot = (data{dataind-1}.position(2) - data{dataind-2}.position(2))/ts;
                dataz_dot = (data{dataind-1}.position(3) - data{dataind-2}.position(3))/ts;
                
                ts_minusone = (data{dataind-2}.TimeStamp - data{dataind-3}.TimeStamp)/10^9;
                datax_dot_minusone = (data{dataind-2}.position(1) - data{dataind-3}.position(1))/ts_minusone;
                datay_dot_minusone = (data{dataind-2}.position(2) - data{dataind-3}.position(2))/ts_minusone;
                dataz_dot_minusone = (data{dataind-2}.position(3) - data{dataind-3}.position(3))/ts_minusone;
                
                datax_2dot = (data{dataind-1}.position(1) - 2*(data{dataind-2}.position(1)) + data{dataind-3}.position(1)) / (ts13mean^2);
                datay_2dot = (data{dataind-1}.position(2) - 2*(data{dataind-2}.position(2)) + data{dataind-3}.position(2)) / (ts13mean^2);
                dataz_2dot = (data{dataind-1}.position(3) - 2*(data{dataind-2}.position(3)) + data{dataind-3}.position(3)) / (ts13mean^2);

%                 datax_2dot = (datax_dot - datax_dot_minusone) / (ts13mean);
%                 datay_2dot = (datay_dot - datay_dot_minusone) / (ts13mean);
%                 dataz_2dot = (dataz_dot - dataz_dot_minusone) / (ts13mean);
            else
                datax_dot = x_dot;
                datay_dot = y_dot;
                dataz_dot = z_dot;
                datax_2dot = x_2dot;
                datay_2dot = y_2dot;
                dataz_2dot = z_2dot;
            end
               
            z = [ data_EM{indexEM}.position(1); data_EM{indexEM}.position(2); data_EM{indexEM}.position(3); datax_dot; datay_dot; dataz_dot];%; datax_2dot;datay_2dot;dataz_2dot]; %measurement       
            x = x_minus + K * (z - (H * x_minus));
            P = (eye(statesize) - K * H ) * P_minus;
            
            rawdata{dataraw,1} = data_EM{indexEM};
            dataraw = dataraw + 1;
            
            data{dataind,1}.position = x(1:3)';
            data{dataind,1}.TimeStamp = t;
            dataind = dataind + 1;
                        
        else
            %no measurement available, only prediction
            A = eye(statesize);
            for i = 1:6
                A(i,i+3) = (timestepfactor*timestepmean)/(10^9);
            end
            for i = 1:3
                A(i,i+6) = .5 * ( (timestepfactor*timestepmean)/(10^9) )^2;
            end
            x_minus = A * x;
            P_minus = A * P * transpose(A) + Q; 
            x = x_minus;
            
            data{dataind,1}.position = x(1:3)';
            data{dataind,1}.TimeStamp = t;
            dataind = dataind + 1;

        end
    end
    timestepmean = timestepcounter * timestepmean + (t - lastTimeStep);
    timestepcounter = timestepcounter +1;
    timestepmean = timestepmean/timestepcounter;
end

for i = 1:size(data,1)
    data{i}.orientation = [.5 .5 .5 .5];
end
for i = 1:size(rawdata,1)
    rawdata{i}.orientation = [.5 .5 .5 .5];
end
raw_cell = trackingdata_to_matrices(rawdata,'cpp');
orig_cell = trackingdata_to_matrices(data_EM, 'cpp');
fromOT_cell = trackingdata_to_matrices(data_EM_by_OT, 'cpp');
data_cell = trackingdata_to_matrices(data, 'cpp');
datafig = Plot_points(data_cell, [], 1, 'o');
Plot_points(fromOT_cell,datafig,2,'+');
Plot_points(orig_cell, datafig, 3, 'x');
Plot_points(raw_cell, datafig, 4, '*');
%Plot_points(orig_cell,[], 3, 'x');
title('data EM: x, data filtered: o');


    end



















