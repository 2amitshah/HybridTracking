function [KalmanData, latestData, RawData_ind, KData_ind] = KalmanUpdate(t, RawData_ind, data, latestData, x, P, Q, H, R, pos_measvar, angle_measvar, statesize, timestep_in_s, KData_ind, KalmanData, estimateOrientation,velocityUpdateScheme, angvelUpdateScheme, KF)

if KData_ind > 1
    x = KalmanData{KData_ind-1,1}.x;
    P = KalmanData{KData_ind-1,1}.P;
end

oldRawData_ind = RawData_ind;

% count how many measurements came in since the last Kalman step
% OT
while(RawData_ind < size(data,1) && ~isempty(data{RawData_ind+1}) && data{RawData_ind+1}.DeviceTimeStamp < t)
    RawData_ind = RawData_ind + 1;
end

% update velocity for measurement update step
x_dot = x(4);
y_dot = x(5);
z_dot = x(6);
if (KData_ind > 2)
    x_dot = (KalmanData{KData_ind-1}.position(1) - KalmanData{KData_ind-2}.position(1)) / timestep_in_s;
    y_dot = (KalmanData{KData_ind-1}.position(2) - KalmanData{KData_ind-2}.position(2)) / timestep_in_s;
    z_dot = (KalmanData{KData_ind-1}.position(3) - KalmanData{KData_ind-2}.position(3)) / timestep_in_s;
end

% update angular velocity for measurement step
x_angvel = x(11);
y_angvel = x(12);
z_angvel = x(13);
if (KData_ind > 2)
    RotTwoLatest = quat2dcm([KalmanData{KData_ind-1}.orientation(1) KalmanData{KData_ind-1}.orientation(2) KalmanData{KData_ind-1}.orientation(3) KalmanData{KData_ind-1}.orientation(4) ;...
                            [KalmanData{KData_ind-2}.orientation(1) KalmanData{KData_ind-2}.orientation(2) KalmanData{KData_ind-2}.orientation(3) KalmanData{KData_ind-2}.orientation(4)]]);
    RotDiff = RotTwoLatest(:,:,1) \ RotTwoLatest(:,:,2); % goes from latest to recent, should be in the right direction
    [x_angle, y_angle, z_angle]= dcm2angle(RotDiff, 'XYZ');
    x_angvel = (x_angle)/timestep_in_s;
    y_angvel = (y_angle)/timestep_in_s;
    z_angvel = (z_angle)/timestep_in_s;
end

Residual=[];
diffTime = timestep_in_s;

%use all measurements of incoming data, loop over oldIndex+1 until
%index
% OT
if(oldRawData_ind~=RawData_ind) % if there were new measurements since the last update
    OnlyPrediction = false;
    for i = oldRawData_ind+1:RawData_ind
        % 
        % update velocity
        if strcmp(velocityUpdateScheme, 'LatestMeasuredData')
            diffTime = data{i}.DeviceTimeStamp - latestData.DeviceTimeStamp;
            if    diffTime < 0.1 ...% maximal one reading was dropped inbetween (assuming 20Hz rate)
               && diffTime > 1000*eps 
                x_dot = (data{i}.position(1) - latestData.position(1)) / diffTime;
                y_dot = (data{i}.position(2) - latestData.position(2)) / diffTime;
                z_dot = (data{i}.position(3) - latestData.position(3)) / diffTime;
            else
                warning(['Local Kalman Position Data is taken to calculate velocity at time: ' num2str(data{i}.DeviceTimeStamp) 's. (RawData index number: ' num2str(i) ')'])
                diffTime = timestep_in_s;
            end   
        end
        R(4:6,4:6) =  2 * pos_measvar * (1/diffTime) * eye(3);
        z = [ data{i}.position(1); data{i}.position(2); data{i}.position(3); x_dot; y_dot; z_dot]; % measurement
        if strcmp(velocityUpdateScheme, 'Inherent')
            z = [ data{i}.position(1); data{i}.position(2); data{i}.position(3)];
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
                else
                    warning(['Local Kalman Attitude Data is taken to calculate angular velocity at time: ' num2str(data{i}.DeviceTimeStamp) 's. (RawData index number: ' num2str(i) ')'])
                    diffTime = timestep_in_s;
                end
                R(11:13,11:13) =  2 * angle_measvar * (1/diffTime) * eye(3);
                z = [ z; data{i}.orientation(4); data{i}.orientation(1); data{i}.orientation(2); data{i}.orientation(3);x_angvel; y_angvel; z_angvel];
            else
                z = [ z; data{i}.orientation(4); data{i}.orientation(1); data{i}.orientation(2); data{i}.orientation(3)];
            end
        end
        % update Kalman timestep
        if(i==(oldRawData_ind+1))
            currentTimestep = (data{i}.DeviceTimeStamp - (t-timestep_in_s));
        else
            currentTimestep = (data{i}.DeviceTimeStamp - data{i-1}.DeviceTimeStamp);
        end
        % update system noise matrix
         % position inaccuracy squared
         Q(1:3,1:3) = diag(repmat(((norm([x(4) x(5) x(6)]) * diffTime)^2)/3,1,3));
         % attitude inaccuracy squared
         Q(7:10,7:10) = diag((angle2quat(x(11)*diffTime, x(12)*diffTime, x(13)*diffTime, 'XYZ')).^2);

        %% Kalman algorithm (KF, EKF, UKF, CKF, ...)

        if (KF==1) && estimateOrientation == 0
            %KF
            % build A
            A = eye(statesize);
            for j = 1:3 %6 
                A(j,j+3) = currentTimestep;
            end
            % state prediction
            x_minus = A * x;
            P_minus = A * P * A' + Q; 
            % state update (correction by measurement)
            K = (P_minus * H') / (H * P_minus * H' + R); %Kalman gain
            x = x_minus + K * (z - (H * x_minus));
            P = (eye(statesize) - K * H ) * P_minus;

            Residual = (z - (H * x_minus));
        else
            %UKF
%                 fstate=@(x)(x + [currentTimestep * x(4); currentTimestep * x(5); currentTimestep * x(6); zeros(statesize-3,1)]);  % nonlinear state equations
            fstate=@(x)transitionFunction_f(x, currentTimestep);
            hmeas=@(x)x(logical(diag(H)));
            [x,P, Residual]=ukf(fstate,x,P,hmeas,z,Q,R);
            % re-normalize quaternion part
            x(7:10)=(quatnormalize(x(7:10)'))';
        end

        latestData = data{i};

    end
    % compute timestep to predict until the end of the time interval
    currentTimestep = (t - data{RawData_ind}.DeviceTimeStamp);        

else % = no measurements arrived inbetween, compute timestep for prediction
    OnlyPrediction = true;
    currentTimestep = timestep_in_s;
    disp(['Only prediction at time: ' num2str(t) 's. (Local Kalman index number: ' num2str(KData_ind) ')'])

    Residual = []; % deviation of prediction and measurement not defined here
end

if (KF==1) && estimateOrientation == 0
    % regular KF
    % build A
    A = eye(statesize);
    for j = 1:3 %6
        A(j,j+3) = currentTimestep;
    end
    % state prediction
    x_minus = A * x;
    P_minus = A * P * A' + Q;
    P = P_minus;
    x = x_minus;
else
    % UKF
    fstate=@(x)transitionFunction_f(x, currentTimestep);
    [x,P]=ukfPrediction(fstate,x,P,Q);
    % re-normalize quaternion part
    x(7:10)=(quatnormalize(x(7:10)'))';
end

%put filtered data into KalmanData struct
KalmanData{KData_ind,1}.position = x(1:3)';
KalmanData{KData_ind,1}.speed = x(4:6);
KalmanData{KData_ind,1}.orientation = x(7:10)';
KalmanData{KData_ind,1}.angvel = x(11:13);
KalmanData{KData_ind,1}.x = x;
KalmanData{KData_ind,1}.P = P;
KalmanData{KData_ind,1}.KalmanTimeStamp = t;
KalmanData{KData_ind,1}.OnlyPrediction = OnlyPrediction;
KalmanData{KData_ind,1}.Deviation = Residual;

KData_ind = KData_ind + 1;


end