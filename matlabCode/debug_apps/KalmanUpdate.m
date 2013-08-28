function [ latestData] = KalmanUpdate(RawDataOT_ind, data, latestData, H, estimateOrientation,velocityUpdateScheme, angvelUpdateScheme)

oldRawDataOT_ind = RawDataOT_ind;
 %use all measurements of incoming data, loop over oldIndex+1 until
    %index
    % OT
if(oldRawDataOT_ind~=RawDataOT_ind) % if there were new measurements since the last update
    OnlyPredictionOT = false;
    for i = oldRawDataOT_ind+1:RawDataOT_ind
        % update velocity
        if strcmp(velocityUpdateScheme, 'LatestMeasuredData')
            diffTime = data{i}.DeviceTimeStamp - latestData.DeviceTimeStamp;
            if    diffTime < 0.1 ...% maximal one reading was dropped inbetween (assuming 20Hz rate)
               && diffTime > 1000*eps 

                x_dot = (data{i}.position(1) - latestData.position(1)) / diffTime;
                y_dot = (data{i}.position(2) - latestData.position(2)) / diffTime;
                z_dot = (data{i}.position(3) - latestData.position(3)) / diffTime;
            else
                warning(['Kalman Position Data (Optical) is taken to calculate velocity at time: ' num2str(data{i}.DeviceTimeStamp) 's. (DataOT index number: ' num2str(i) ')'])
            end   
        end
        z_OT = [ data{i}.position(1); data{i}.position(2); data{i}.position(3); x_dot; y_dot; z_dot]; % measurement

        if strcmp(velocityUpdateScheme, 'Inherent')
            z_OT = [ data{i}.position(1); data{i}.position(2); data{i}.position(3)];
        end

        if estimateOrientation == 1
            if ~strcmp(angvelUpdateScheme, 'Inherent')
                diffTime = data{i}.DeviceTimeStamp - latestData.DeviceTimeStamp;
                if    diffTime < 0.1 ...% maximal one reading was dropped inbetween (assuming 20Hz rate)
                   && diffTime > 1000*eps
                    % convert 2 quaternions to differential explicit XYZ-Euler via DCM
                    % to avoid singularities
                    RotTwoLatest = quat2dcm( [data{i}.orientation(4) data{i}.orientation(1) data{i}.orientation(2) data{i}.orientation(3) ;...
                                [latestData.orientation(4) latestData.orientation(1) latestData.orientation(2) latestData.orientation(3) ]]);
                    RotDiff = RotTwoLatest(:,:,1) \ RotTwoLatest(:,:,2); % goes from latest to recent, should be in the right direction
                    [x_angle, y_angle, z_angle]= dcm2angle(RotDiff, 'XYZ');
                    x_angvel = (x_angle)/diffTime;
                    y_angvel = (y_angle)/diffTime;
                    z_angvel = (z_angle)/diffTime;
                end
                z_OT = [ z_OT; data{i}.orientation(4); data{i}.orientation(1); data{i}.orientation(2); data{i}.orientation(3);x_angvel; y_angvel; z_angvel];
            else
                z_OT = [ z_OT; data{i}.orientation(4); data{i}.orientation(1); data{i}.orientation(2); data{i}.orientation(3)];
            end
        end

        if(i==(oldRawDataOT_ind+1))
            currentTimestep = (data{i}.DeviceTimeStamp - (t-timestep_in_s));
        else
            currentTimestep = (data{i}.DeviceTimeStamp - data{i-1}.DeviceTimeStamp);
        end
        % build A
        A_OT = eye(statesize);
        for j = 1:3 %6 
            A_OT(j,j+3) = currentTimestep;
        end

        %% Kalman algorithm (KF, EKF, UKF, CKF, ...)

        if (KF==1) && estimateOrientation == 0
            %KF
            % state prediction
            x_minus_OT = A_OT * x_OT;
            P_minus_OT = A_OT * P_OT * A_OT' + Q_OT; 
            % state update (correction by measurement)
            K_OT = (P_minus_OT * H_OT') / (H_OT * P_minus_OT * H_OT' + R_OT); %Kalman gain
            x_OT = x_minus_OT + K_OT * (z_OT - (H_OT * x_minus_OT));
            P_OT = (eye(statesize) - K_OT * H_OT ) * P_minus_OT;

            DeviationOT = (z_OT - (H_OT * x_minus_OT));
        else
            %UKF
%                 fstate_OT=@(x)(x + [currentTimestep * x(4); currentTimestep * x(5); currentTimestep * x(6); zeros(statesize-3,1)]);  % nonlinear state equations
            fstate_OT=@(x)transitionFunction_f(x, currentTimestep);
            hmeas_OT=@(x)x(logical(diag(H)));
            [x_OT,P_OT, DeviationOT]=ukf(fstate_OT,x_OT,P_OT,hmeas_OT,z_OT,Q_OT,R_OT);
        end

        latestData = data{i};

    end
    % compute timestep to predict until the end of the time interval
    currentTimestep = (t - data{RawDataOT_ind}.DeviceTimeStamp);        

else % = no measurements arrived inbetween, compute timestep for prediction
    OnlyPredictionOT = true;
    currentTimestep = timestep_in_s;
    disp(['Only prediction at time: ' num2str(t) 's. (Optical kalman index number: ' num2str(KDataOT_ind) ')'])

    DeviationOT = []; % deviation of prediction and measurement not defined here
end

% build A
A_OT = eye(statesize);
for j = 1:3 %6
    A_OT(j,j+3) = currentTimestep;
end
% state prediction
x_minus_OT = A_OT * x_OT;
P_minus_OT = A_OT * P_OT * A_OT' + Q_OT;
P_OT = P_minus_OT;
x_OT = x_minus_OT;

%put filtered data into KalmanData struct
KalmanDataOT{KDataOT_ind,1}.position = x_OT(1:3)';
KalmanDataOT{KDataOT_ind,1}.speed = [x_dot; y_dot; z_dot];
KalmanDataOT{KDataOT_ind,1}.P = P_OT;
KalmanDataOT{KDataOT_ind,1}.KalmanTimeStamp = t;
KalmanDataOT{KDataOT_ind,1}.OnlyPrediction = OnlyPredictionOT;
KalmanDataOT{KDataOT_ind,1}.Deviation = DeviationOT;

KDataOT_ind = KDataOT_ind + 1;




end