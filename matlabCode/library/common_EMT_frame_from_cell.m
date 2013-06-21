function [H_commonEMT_to_EMCS, H_EMCS_to_commonEMT] = common_EMT_frame_from_cell(H_EMT_to_EMCS_cell, verbosity)

if ~exist('verbosity', 'var')
    verbosity = 'vDebug';
end

numPts = size(H_EMT_to_EMCS_cell{1},3);
numSen = size(H_EMT_to_EMCS_cell,2);

%new version
if numSen > 1
    % plot position data
    if strcmp(verbosity,'vDebug')
    figurehandle = Plot_points(H_EMT_to_EMCS_cell(2:end),[], 2);
    Plot_points(H_EMT_to_EMCS_cell(1), figurehandle, 1); %EMT1 is blue
    title('Interpolated position of EM sensors')
    end

    % get average EMT H_differences
    H_diff=cell(1,numSen-1);

    for j=2:numSen
        errorPoints = 0;
        for i=1:numPts
            %calculate position of sensors 2, 3, etc relative to sensor 1
            %check translations in these matrices.. if any of both is
            %bad: don't add to H_diff
            %check if a point exists for the wished timestamp
            if ( ( abs(H_EMT_to_EMCS_cell{1}(1,4,i)) > 10000 ) || ( abs(H_EMT_to_EMCS_cell{j}(1,4,i)) > 10000 )  ...
                    || ( H_EMT_to_EMCS_cell{1}(1,4,i) == 0 || H_EMT_to_EMCS_cell{j}(1,4,i) == 0) )
                % point invalid
                errorPoints = errorPoints+1;
            else    
                H_diff{j-1}(:,:,i-errorPoints) = inv(H_EMT_to_EMCS_cell{1}(:,:,i))*H_EMT_to_EMCS_cell{j}(:,:,i);
            end
        end
        H_diff{j-1} = mean_transformation(H_diff{j-1});
    end
    
% save('H_EMTx_to_EMT1.mat', 'H_diff')

    % project every EMT 2 etc to EMT 1, build average
    frameWithoutError = zeros(4,4,1);
    errorPoints = 0;
    H_new = cell(1,numSen-1);
    goodSens_array = zeros(1,numPts);
    
    for i=1:numPts
        collectframe = zeros(4);
        goodSens = 0;
        if ( abs(H_EMT_to_EMCS_cell{1}(1,4,i)) > 10000 ||  H_EMT_to_EMCS_cell{1}(1,4,i) == 0)
            % point invalid
            disp 'invalid point'
        else
            goodSens = goodSens + 1;
            collectframe(:,:,goodSens) = H_EMT_to_EMCS_cell{1}(:,:,i);
        end
        
        for j=2:numSen
            if ( abs(H_EMT_to_EMCS_cell{j}(1,4,i)) > 10000 ||  H_EMT_to_EMCS_cell{j}(1,4,i) == 0)
                % point invalid
                disp 'invalid point'
            else            
                H_new{j-1} = H_EMT_to_EMCS_cell{j}(:,:,i)*inv(H_diff{j-1});
                goodSens = goodSens + 1;
                collectframe(:,:,goodSens) = H_new{j-1};
            end
        end
        goodSens_array(i)=goodSens;
        % new and nice mean value creation
        if (goodSens == 0) %in case no sensor is good: no new entry in frameWithoutError,
                           %same entry again in data_EM_common..?
            errorPoints = errorPoints + 1;
        else
            frameWithoutError(:,:,i-errorPoints) = mean_transformation(collectframe);
        end
    end
    %plot number of used sensors per position
    if strcmp(verbosity,'vDebug')
    numberOfSensors_fig = figure;
    title('Number of EM sensors used to compute common frame')
    hold on
    plot(goodSens_array, 'x');
    hold off
    end
    
    H_commonEMT_to_EMCS = frameWithoutError;

    % plot position data of synthesized position
    if strcmp(verbosity,'vDebug')
    wrappercell{1}=H_commonEMT_to_EMCS;
    hold on
    SensorPosition_fig = Plot_points(wrappercell, [], 1);%synth. data is blue
    Plot_points(H_EMT_to_EMCS_cell,SensorPosition_fig,2);
    hold off
    title('Original position of EM sensors and computed common frame (blue)')
    end
    
else
    H_commonEMT_to_EMCS = H_EMT_to_EMCS_cell{1};
end
    numPts = size(H_commonEMT_to_EMCS,3);
    H_EMCS_to_commonEMT = zeros(4,4,numPts);
    for i=1:numPts
        H_EMCS_to_commonEMT(:,:,i) = inv(H_commonEMT_to_EMCS(:,:,i));
    end
end
