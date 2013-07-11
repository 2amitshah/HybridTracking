% this function is for combining the data of the two sensors. Tthis means that
% we use the OT data (at the position of EM) when it is available because it is usually more
% accurate and the EM data (corrected by the known distortion field)
% otherwise. In order to overcome jittering and also small changes at the
% points where we change from OT to EM and back we use a Kalman filter for
% smoothing and correcting.

function datafiltered = common_OT_EM_kalman(path, testrow_name_EMT, testrow_name_OT, H_OT_to_EMT)

close all;
if ~exist('path','var')
     pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
     path = [pathGeneral filesep 'measurements' filesep '06.13_Measurements' filesep '02'];
 end
 if ~exist('H_OT_to_EMT','var')
     load(which('H_OT_to_EMT.mat'));
 end
 
 if ~exist('testrow_name_EMT','var')
    testrow_name_EMT = 'EMTrackingcont_1';
 end
 
 if ~exist('testrow_name_OT','var')
    testrow_name_OT = 'OpticalTrackingcont_1';
 end
 
frequency = 40;
% get Y, equal to EMCS_to_OCS
Y = polaris_to_aurora(path, H_OT_to_EMT,'cpp','static','vRelease');
 
%% get the improved position of EM 1 and EM 2 (and EM 3, if available) at the position of EM 1
% (data_EM_common) and the data of OT (data_OT_common) at the same synthetic timestamps

[~, ~, data_EM_common_tmp, data_OT_common_tmp] =  OT_common_EMT_at_synthetic_timestamps_distortion_correction(path, testrow_name_EMT,testrow_name_OT, frequency, 'vRelease');

% mysize = 150;
% data_EM_common = cell(mysize,1);
% data_OT_common = cell(mysize,1);
% for i = 1:mysize
%    data_EM_common{i} = data_EM_common_tmp{i+1100};
%    data_OT_common{i} = data_OT_common_tmp{i+1100};
% end
data_EM_common = data_EM_common_tmp;
data_OT_common = data_OT_common_tmp;
%data_EM_synth = syntheticPositions();

%prepare data
numPts = size(data_EM_common,1);
numSensors = 2;
mat=cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

% Relevant matrix for computing transformations
[H_EMT_to_EMCS] = trackingdata_to_matrices(data_EM_common, 'CppCodeQuat');
[H_OT_to_OCS] = trackingdata_to_matrices(data_OT_common, 'CppCodeQuat');
H_OT_to_OCS = H_OT_to_OCS{1,1};
H_EMT_to_EMCS = H_EMT_to_EMCS{1,1};


%% calculate where EM tracker should be
H_EMT_to_OT = inv(H_OT_to_EMT);
H_EMT_to_EMCS_by_OT = zeros(4,4,numPts);
H_diff_EMT_to_EMCS = zeros(4,4,numPts);
translation_EMTcell = cell(1,2);

data_EM_common_by_OT = cell(numPts,1);

for i = 1:numPts
    H_OT_to_EMCS = Y*H_OT_to_OCS(:,:,i);
    H_EMT_to_EMCS_by_OT(:,:,i) = H_OT_to_EMCS * H_EMT_to_OT; %data_EM_common_by_OT
    data_EM_common_by_OT{i}.valid = 0;
    if(data_OT_common{i}.valid == 1 && data_EM_common{i}.valid == 1) %DEBUG: here we limit data_EM_common_by_OT{i}.valid to only be valid when OT AND EMT at that time are valid.
        H_diff_EMT_to_EMCS(:,:,i) = inv(H_EMT_to_EMCS(:,:,i))*H_EMT_to_EMCS_by_OT(:,:,i);
        translation_EMTcell{1}.vector(:,i) = H_EMT_to_EMCS_by_OT(1:3,4,i);
        translation_EMTcell{2}.vector(:,i) = H_EMT_to_EMCS(1:3,4,i);
    
        data_EM_common_by_OT{i}.TimeStamp = data_OT_common{i}.TimeStamp;      
        data_EM_common_by_OT{i}.position = transpose(H_EMT_to_EMCS_by_OT(1:3,4,i)) ;
        data_EM_common_by_OT{i}.orientation = transpose(rot2quat_q41(H_EMT_to_EMCS_by_OT(1:3, 1:3, i)));   
        data_EM_common_by_OT{i}.valid = data_OT_common{i}.valid;
    end
end

%% combine the data of the two sensors
data = data_EM_common_by_OT;
for i = 1:numPts
    if (~(data{i}.valid) && data_EM_common{i}.valid)
        data{i} = data_EM_common{i};
        data{i}.byOT = 0;
    else
        data{i}.byOT = 1;
    end    
end


%% do Kalman filtering with data
timestep_in_s = (data{2}.TimeStamp - data{1}.TimeStamp) / 10^9;

% initialize matrices and vectors

% initial state vector init_x, start with timestep 3 (in order to have the first and second
% derivative)
x_dot = (data{3}.position(1) - data{2}.position(1))/timestep_in_s;
y_dot = (data{3}.position(2) - data{2}.position(2))/timestep_in_s;
z_dot = (data{3}.position(3) - data{2}.position(3))/timestep_in_s;
x_2dot = (x_dot - ((data{2}.position(1) - data{1}.position(1))/timestep_in_s)) / timestep_in_s;
y_2dot = (y_dot - ((data{2}.position(2) - data{1}.position(2))/timestep_in_s)) / timestep_in_s;
z_2dot = (z_dot - ((data{2}.position(3) - data{1}.position(3))/timestep_in_s)) / timestep_in_s;

initx = [data{3}.position(1); data{3}.position(2); data{3}.position(3); x_dot; y_dot; z_dot; x_2dot; y_2dot; z_2dot ] 

statesize = 9;
observationsize = 3;
% state transition matrix A
A = eye(statesize);
for i = 1:6
    A(i,i+3) = timestep_in_s;
end
for i = 1:3
    A(i,i+6) = .5 * timestep_in_s^2;
end
A

H = zeros(observationsize,statesize);
H(1:observationsize, 1:observationsize) = eye(observationsize,observationsize);
H

% estimate measurement noise covariance matrix R using some of the first measurements
% amountForCov = 150;
% firstVals = zeros(amountForCov,size(initx));
% for i = 1:amountForCov
%    firstVals(i,1) = data{i+2}.position(1);
%    firstVals(i,2) = data{i+2}.position(2);
%    firstVals(i,3) = data{i+2}.position(3);
%    x_dot = (data{i+2}.position(1) - data{i+1}.position(1))/timestep_in_s;
%    y_dot = (data{i+2}.position(2) - data{i+1}.position(2))/timestep_in_s;
%    z_dot = (data{i+2}.position(3) - data{i+1}.position(3))/timestep_in_s;
%    firstVals(i,4) = x_dot;
%    firstVals(i,5) = y_dot;
%    firstVals(i,6) = z_dot;
%    x_2dot = (x_dot - ((data{i+1}.position(1) - data{i}.position(1))/timestep_in_s)) / timestep_in_s;
%    y_2dot = (y_dot - ((data{i+1}.position(2) - data{i}.position(2))/timestep_in_s)) / timestep_in_s;
%    z_2dot = (z_dot - ((data{i+1}.position(3) - data{i}.position(3))/timestep_in_s)) / timestep_in_s;
%    firstVals(i,7) = x_2dot;
%    firstVals(i,8) = y_2dot;
%    firstVals(i,9) = z_2dot;   
% end
% firstVals
%R_OT = cov(firstVals);
%R_OT = R_OT(1:observationsize,1:observationsize);
%r=0.03;    %std of measurement 

%r=sqrt(.01);    %std of measurement 
%R_OT=r^2;%*eye(3);
%R_OT=.1*eye(3);

% estimate process noise covariance matrix Q
%q=0.1;    %std of process 
%Q=q^2*eye(statesize);

% initial state covariance (P)
initV = .25*eye(statesize);
%velocity error
for i = 1:3
    initV(i+3,i+3) = 2*(initV(i+3,i+3))/timestep_in_s;
end
%acceleration error
for i = 1:3
    initV(i+6,i+6) = 4*(initV(i+3,i+3))/(timestep_in_s^2);
end

datafiltered = cell(numPts-3,1);

% observation at time t, coming from data
y = zeros(3,numPts-3);
for i = 1:numPts-3
    if(data{i+3}.valid)
        y(1:3,i) = data{i+3}.position;
    else
        y(1:3,i) = y(1:3,i-1);
    end
   %y(2,i)
   %y(3,i)
end


%Q = 0.1*eye(statesize);
%R = 1*eye(observationsize);

% the Polaris system measures with an accuracy of 0.25 mm, so I just try
% out this value here. But in velocity and acc error, we have to put much
% greater values :( since they are derived from erroneous positions and
% then divided by a value
Q = .25*eye(statesize);
R = .25*eye(observationsize);

H_mine = H;
Q_mine = Q;
R_mine = R;
x_mine = initx;
A_mine = A;
P_mine = initV;

[xfilt, Vfilt, VVfilt, loglik] = kalman_filter(y, A, H, Q, R, initx, initV);

for i = 4:numPts
    datafiltered{i}.position = (xfilt(1:observationsize,i-3))';
    datafiltered{i}.speed = (xfilt(4:6,i-3))';
    datafiltered{i}.acceleration = (xfilt(7:9,i-3))';
end


%% kalman - my implementation

datafiltered_mine = cell(numPts-3,1);
for i = 4:numPts %loop over all measurements (starting from the fourth) for filtering          
    %% time update (prediction)
    x_minus_mine = A_mine * x_mine;
    P_minus_mine = A_mine * P_mine * transpose(A_mine) + Q_mine; 

    %% measurement update (correction)
    
    if data{i}.valid

       
        
%         if(data{i-1}.valid && data{i-2}.valid)
%             R_mine = .1*eye(statesize);
%             H_mine = eye(statesize);
%             K_mine = P_minus_mine * H_mine' * ((H_mine * P_minus_mine * H_mine' + R_mine)^-1); %Kalman gain
%             x_dot = (data{i}.position(1) - data{i-1}.position(1))/timestep_in_s;
%             y_dot = (data{i}.position(2) - data{i-1}.position(2))/timestep_in_s;
%             z_dot = (data{i}.position(3) - data{i-1}.position(3))/timestep_in_s;
%             x_2dot = (x_dot - ((data{i-1}.position(1) - data{i-2}.position(1))/timestep_in_s)) / timestep_in_s;
%             y_2dot = (y_dot - ((data{i-1}.position(2) - data{i-2}.position(2))/timestep_in_s)) / timestep_in_s;
%             z_2dot = (z_dot - ((data{i-1}.position(3) - data{i-2}.position(3))/timestep_in_s)) / timestep_in_s;
%             z_mine = [ data{i}.position(1); data{i}.position(2); data{i}.position(3); x_dot; y_dot; z_dot; x_2dot; y_2dot; z_2dot]; %measurement
%             x_mine = x_minus_mine + K_mine * (z_mine - x_minus_mine);
%             P_mine = (eye(size(x_mine,1)) - K_mine * H_mine ) * P_minus_mine;
%         else
%             R_mine = .1*eye(observationsize);
%             H_mine = H;
            
%         end
        
        K_mine = P_minus_mine * H_mine' * ((H_mine * P_minus_mine * H_mine' + R_mine)^-1); %Kalman gain
        z_mine = [ data{i}.position(1); data{i}.position(2); data{i}.position(3)]; %measurement      
        x_mine = x_minus_mine + K_mine * (z_mine - (H_mine * x_minus_mine));
        P_mine = (eye(statesize) - K_mine * H_mine ) * P_minus_mine;

    else
        if i > 6
            x_mine = x_minus_mine;
            % decrease the trust in the measurements .. don't go back to the reals measurements too quickly when new valid points are available
            K_mine = .9 * K_mine;
            P_minus_mine = .9 * P_minus_mine; 
            P_mine = (eye(statesize) - K_mine * H_mine ) * P_minus_mine;
            % decrease the speed (simple version of friction :) )
%             x_mine(4) = .9*x_mine(4); 
%             x_mine(5) = .9*x_mine(5);
%             x_mine(6) = .9*x_mine(6);
            %use the last speed data for the new position..
%             x_mine(1) = x_mine(1) + (datafiltered_mine{i-1}.position(1) - datafiltered_mine{i-2}.position(1));
%             x_mine(2) = x_mine(2) + (datafiltered_mine{i-1}.position(2) - datafiltered_mine{i-2}.position(2));
%             x_mine(3) = x_mine(3) + (datafiltered_mine{i-1}.position(3) - datafiltered_mine{i-2}.position(3));
        end
    end

            

    datafiltered_mine{i}.position(1) = x_mine(1);
    datafiltered_mine{i}.position(2) = x_mine(2);
    datafiltered_mine{i}.position(3) = x_mine(3);
    %datafiltered_mine{i}.speed_norm = norm([x_mine(4);x_mine(5);x_mine(6)]);
    datafiltered_mine{i}.speed(1) = x_mine(4);
    datafiltered_mine{i}.speed(2) = x_mine(5);
    datafiltered_mine{i}.speed(3) = x_mine(6);
    %datafiltered_mine{i}.acceleration_norm = norm([x_mine(7);x_mine(8);x_mine(9)]); 
    datafiltered_mine{i}.acceleration(1) = x_mine(7);
    datafiltered_mine{i}.acceleration(2) = x_mine(8);
    datafiltered_mine{i}.acceleration(3) = x_mine(9);
    datafiltered_mine{i}.K = K_mine;
    if data{i}.valid
        datafiltered_mine{i}.zminHx = (z_mine - (H_mine * x_minus_mine)); %if zminHx is small: prediction was the same as measurement.. if we don't have a measurement, we set zminHx = 100
    else
        datafiltered_mine{i}.zminHx = [100;100;100];
    end
    datafiltered_mine{i}.valid = 1;     
%     if(i > 6)
%         x_mine(4) = datafiltered_mine{i}.position(1) - datafiltered_mine{i-1}.position(1);
%         x_mine(5) = datafiltered_mine{i}.position(2) - datafiltered_mine{i-1}.position(2);
%         x_mine(6) = datafiltered_mine{i}.position(3) - datafiltered_mine{i-1}.position(3);
%     end        
end

%% plots
close all;


for i = 4:numPts    
   datafiltered{i}.orientation = [.5 .5 .5 .5];
   datafiltered_mine{i}.orientation = [.5 .5 .5 .5];
end

datafiltered_cell = trackingdata_to_matrices(datafiltered, 'cpp');
datafilteredmine_cell = trackingdata_to_matrices(datafiltered_mine, 'cpp');

kalmancomparefigure = Plot_points(datafiltered_cell, [], 1, 'o');
Plot_points(datafilteredmine_cell, kalmancomparefigure, 3, 'x');
title('kalman (o) and my kalman (x)');



figure
subplot(2,3,1)
hold on
for i = 4:numPts
    if data{i}.valid
        plot3(data{i}.position(1),data{i}.position(2),data{i}.position(3),'rx');
    end    
    plot3(datafiltered{i}.position(1),datafiltered{i}.position(2),datafiltered{i}.position(3),'bx');
    xf(i-3) = datafiltered{i}.position(1);
    yf(i-3) = datafiltered{i}.position(2);
    zf(i-3) = datafiltered{i}.position(3);  
end
line(xf,yf,zf);
xlabel('x')
ylabel('y')
zlabel('z')    
axis image vis3d
set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
view(3)
hold off

%plotEnvironment(1, H_OT_to_EMT, Y)
title('Position of common EMT1 sensor (red, filtered: blue) at synthetic timestamps')

subplot(2,3,2)
hold on
for i = 4:numPts
    plot(i,datafiltered{i}.speed(1),'r');
    plot(i,datafiltered{i}.speed(2),'g');
    plot(i,datafiltered{i}.speed(3),'b');        
end
title('speed of filtered signal in x-, y- and z-direction (r,g,b)');
hold off

subplot(2,3,3)
hold on
for i = 4:numPts
    plot(i,datafiltered{i}.acceleration(1),'r');
    plot(i,datafiltered{i}.acceleration(2),'g');
    plot(i,datafiltered{i}.acceleration(3),'b');        
end
title('acceleration of filtered signal in x-, y- and z-direction (r,g,b)');
hold off


subplot(2,3,4)
hold on
for i = 4:numPts
    if data{i}.valid
        plot3(data{i}.position(1),data{i}.position(2),data{i}.position(3),'rx');
    end
    plot3(datafiltered_mine{i}.position(1),datafiltered_mine{i}.position(2),datafiltered_mine{i}.position(3),'bx');
    xf(i-3) = datafiltered_mine{i}.position(1);
    yf(i-3) = datafiltered_mine{i}.position(2);
    zf(i-3) = datafiltered_mine{i}.position(3);  
end
line(xf,yf,zf);
%plotEnvironment(1, H_OT_to_EMT, Y)
xlabel('x')
ylabel('y')
zlabel('z')    
axis image vis3d
set(gca,'ZDir','reverse')
set(gca,'YDir','reverse')
view(3)
title('mine: Position of common EMT1 sensor (red, filtered: blue) at synthetic timestamps')
hold off

subplot(2,3,5)
hold on
for i = 4:numPts
    plot(i,datafiltered_mine{i}.speed(1),'r');
    plot(i,datafiltered_mine{i}.speed(2),'g');
    plot(i,datafiltered_mine{i}.speed(3),'b');        
end
title('mine: speed of filtered signal in x-, y- and z-direction (r,g,b)');
hold off

subplot(2,3,6)
hold on
for i = 4:numPts
    plot(i,datafiltered_mine{i}.acceleration(1),'r');
    plot(i,datafiltered_mine{i}.acceleration(2),'g');
    plot(i,datafiltered_mine{i}.acceleration(3),'b');        
end
title('mine: acceleration of filtered signal in x-, y- and z-direction (r,g,b)');
hold off



figure(3)
hold on 
for i = 4:numPts
   plot(i, norm(datafiltered{i}.position - datafiltered_mine{i}.position),'bx');
end
hold off
title('difference between the two implementations of the kalman filter');

figure(4)
hold on 
for i = 4:numPts
   plot(i, datafiltered_mine{i}.K(1,1),'b');
   plot(i, datafiltered_mine{i}.K(4,1),'r');
   plot(i, datafiltered_mine{i}.K(7,1),'g');
end
hold off
title('development of the kalman gain over time, K(1,1): blue, K(4,1): red, K(7,1): green')
end



















