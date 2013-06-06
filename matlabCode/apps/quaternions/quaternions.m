%%%%%%%%%%%%%
%Caution:
%this read in procedure is only applicable to one EMT and one OT dataset,
%in that way, that the EM dataset can contain various EM Trackers and the
%Optical data only contains one OT.
%calibration is done for Optical to the first EM tracker.
%%%%%%%%%%%%%
%% data read in
% do preparation
clear variables globals;
close all;

pathGeneral = fileparts(fileparts(fileparts(fileparts(which(mfilename)))));
path = [pathGeneral filesep 'measurements' filesep 'testmfrom_NDItrack'];
testrow_name_EMT = 'hybridEMT';
testrow_name_OT = 'hybridOT';

% get data for hand/eye calib
[data_EMT] = read_NDI_tracking_files(path, testrow_name_EMT);
[data_OT] = read_NDI_tracking_files(path, testrow_name_OT);

numPts = size(data_EMT,1);
numSensors = 1;
numUnknowns = 6;
mat=cell(numPts-1,numSensors);
points = cell(numPts,numSensors);

% a is EMT
% b is OT
reset(symengine)

for i = 1:numPts
    for j = 1:numSensors
        points{i,j}.Ha = getMatrixH((data_EMT{i,j}.orientation(2:4))',data_EMT{i,j}.position');
        points{i,j}.Hb = getMatrixH((data_OT{i}.orientation(2:4))',data_OT{i}.position');                
    end
end

for i = 1:numPts-1
    for j = 1:numSensors
        
        
        % First we have to compute the motion!!!
        [mat{i,j}.aRotation,mat{i,j}.aTranslation] = obtainQuatMotion(points{i,j}.Ha, points{i+1,j}.Ha);
        [mat{i,j}.bRotation,mat{i,j}.bTranslation] = obtainQuatMotion(points{i,j}.Hb, points{i+1,j}.Hb);
        
        mat{i,j}.aPrime = 0.5*mat{i,j}.aTranslation.*mat{i,j}.aRotation;
        mat{i,j}.bPrime = 0.5*mat{i,j}.bTranslation.*mat{i,j}.bRotation;
        
        mat{i,j}.skew = skew(mat{i,j}.aRotation+mat{i,j}.bRotation);
        mat{i,j}.skewPrime = skew(mat{i,j}.aPrime+mat{i,j}.bPrime);
        
        
        
        %options = optimset('algorithm', 'trust-region-reflective', 'TolFun', 1e-20);
        
        
        syms rx ry rz tx ty tz;
        mat{i,j}.q = [0; rx; ry; rz];
        mat{i,j}.qPrime = [0; 0.5*tx*rx; 0.5*ty*ry; 0.5*tz*rz];
        clear rx ry rz tx ty tz;
        
        mat{i,j}.x = [mat{i,j}.q; mat{i,j}.qPrime];
        
        % Definition of matrix
        mat{i,j}.C =    [mat{i,j}.aRotation-mat{i,j}.bRotation mat{i,j}.skew zeros(3,1) zeros(3,3);...
                         mat{i,j}.aPrime-mat{i,j}.bPrime mat{i,j}.skewPrime mat{i,j}.aRotation-mat{i,j}.bRotation mat{i,j}.skew];
        
        mat{i,j}.Cx = (mat{i,j}.C)*(mat{i,j}.x);
        mat{i,j}.constraints = [dot(conj(mat{i,j}.q),mat{i,j}.q)-1;...
                                dot(conj(mat{i,j}.q),mat{i,j}.qPrime) + dot(conj(mat{i,j}.qPrime),mat{i,j}.q)];
        
%         mat{i,j}.constraints = [mat{i,j}.qt*mat{i,j}.q-1;...
%                                 mat{i,j}.qt*mat{i,j}.qp];
        mat{i,j}.equations = [mat{i,j}.Cx; mat{i,j}.constraints];
        
        %x = lsqnonlin(@(x)myFunc1src(time, ch0, chs, x, f, r), x0, zeros(1,4),...
        %[1e-4 1e8 1e-6 1e9], options);
%         mat{i,j}.solution = solve(mat{i,j}.equations == zeros(8,1));
%         
%         mat{i,j}.solutionNum = zeros(numUnknowns,2);
%         mat{i,j}.solutionNum = [double(mat{i,j}.solution.rx)';double(mat{i,j}.solution.ry)';double(mat{i,j}.solution.rz)';
%                                 double(mat{i,j}.solution.tx)';double(mat{i,j}.solution.ty)';double(mat{i,j}.solution.tx)'];
            
    end
    
end


%% OPTIMIZER

fh = @(x) objectiveFunction(x, mat, numPts, numSensors);
options = optimset('TolX',1e-8, 'MaxFunEvals', 10000, 'MaxIter', 10000);
x0 = [0; -0.85; -0.45; -10; -2; -49];
lowBnd = [-1; -1; -1; -100; -100; -100];
uppBnd = [1; 1; 1; 100; 100; 100];
%options = optimset('algorithm', 'levenberg-marquardt', 'TolFun', 1e-20);
 [transform_params, min_value,exitflag] = fminsearchbnd(fh,x0,lowBnd,uppBnd,options);
%[transform_params, min_value,exitflag] = fminsearch(fh,x0,options);



%x = lsqnonlin(@(x)objectiveFunction(x, mat, numPts, numSensors), [1; 1; 1; 100; 100; 100], zeros(6,1), [1; 1; 1; 1e3; 1e3; 1e3], options);
        %[1e-4 1e8 1e-6 1e9], options);


%%
clear qx qy qz tx ty tz;
solutions = zeros(numPts-1,numUnknowns*numSensors*2);
for i = 1:numPts-1
    final=0;
    for j = 1:numSensors
        for k = 1:numUnknowns
            solutions(i,final+k*2-1:final+k*2)=mat{i,j}.solutionNum(k,:);
        end
        final=2*(final+numUnknowns)+1;
    end
end


% 
% %%
% %gHc = tracking_handEyeCalib(bHg, wHc)
% 
% % We want to have EMCS as world coordinates. World in Tsai's setup always
% % means the grid. I will filp this to match EMT~Cam and OT~Marker.
% for i = 1:numPts
%     Hgrid2cam(:,:,i)        = inv(mat{1}(:,:,i)); %(=EMCS_to_EMT)
%     H_EMT_to_EMCS(:,:,i)    = mat{1}(:,:,i);
%     Hmarker2world(:,:,i)    = mat{2}(:,:,i);      %(=OT_to_OCS)
% end
% 
% %% plot position data
% % plot to see how positions were recorded
% for i = 1:numPts
%     %optical
%     %rotation
%     xaxes_optical(i,:) = Hmarker2world(1:3,1,i)';
%     yaxes_optical(i,:) = Hmarker2world(1:3,2,i)';
%     zaxes_optical(i,:) = Hmarker2world(1:3,3,i)';
%     %translation
%     opticalPoints(:,i) = Hmarker2world(1:3,4,i);
%     
%     %em tracker
%     %rotation
%     xaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,1,i)';
%     yaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,2,i)';
%     zaxes_emFirst(i,:) = H_EMT_to_EMCS(1:3,3,i)';
%     %translation
%     emPointsFirstSensor(:,i) = H_EMT_to_EMCS(1:3,4,i);
% end
% c = colormap('lines');
% 
% % plot all OT positions
% figure(1)
% hold on
% plot3(opticalPoints(1,:), opticalPoints(2,:), opticalPoints(3,:), 'x', 'Color', c(1,:) );
% hold off
% title({'Optical Center position in optical coordinate system OCS',...
%     'orientation is shown in XYZ = RGB'})
% xlabel('x')
% ylabel('y')
% zlabel('z')
% 
% % plot all OT orientations
% %x axis is red, y axis is green, z axis is blue
% hold on
% line([opticalPoints(1,:); opticalPoints(1,:)+30*xaxes_optical(:,1)'],...
%     [opticalPoints(2,:); opticalPoints(2,:)+30*xaxes_optical(:,2)'],...
%     [opticalPoints(3,:); opticalPoints(3,:)+30*xaxes_optical(:,3)'], 'LineWidth',3,'Color', [1 0 0]);
% hold off
% hold on
% line([opticalPoints(1,:); opticalPoints(1,:)+30*yaxes_optical(:,1)'],...
%     [opticalPoints(2,:); opticalPoints(2,:)+30*yaxes_optical(:,2)'],...
%     [opticalPoints(3,:); opticalPoints(3,:)+30*yaxes_optical(:,3)'], 'LineWidth',3,'Color', [0 1 0]);
% hold off
% hold on
% line([opticalPoints(1,:); opticalPoints(1,:)+30*zaxes_optical(:,1)'],...
%     [opticalPoints(2,:); opticalPoints(2,:)+30*zaxes_optical(:,2)'],...
%     [opticalPoints(3,:); opticalPoints(3,:)+30*zaxes_optical(:,3)'], 'LineWidth',3,'Color', [0 0 1]);
% hold off
% axis image vis3d
% 
% % plot all EMT positions
% figure(2)
% hold on
% plot3(emPointsFirstSensor(1,:), emPointsFirstSensor(2,:),emPointsFirstSensor(3,:), 'x', 'Color', c(1,:) );
% hold off
% title({'EM Tracker position in electromagnetical coordinate system (EMCS)',...
%     'orientation is shown in XYZ = RGB',...
%     'transformed OT position in EMCS is shown as red x, corresponding positions are connected'})
% xlabel('x')
% ylabel('y')
% zlabel('z')
% 
% % plot all EMT orientations of EM tracker 1
% 
% hold on
% line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+30*xaxes_emFirst(:,1)'],...
%     [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+30*xaxes_emFirst(:,2)'],...
%     [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+30*xaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [1 0 0]);
% hold off
% hold on
% line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+30*yaxes_emFirst(:,1)'],...
%     [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+30*yaxes_emFirst(:,2)'],...
%     [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+30*yaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [0 1 0]);
% hold off
% hold on
% line([emPointsFirstSensor(1,:); emPointsFirstSensor(1,:)+30*zaxes_emFirst(:,1)'],...
%     [emPointsFirstSensor(2,:); emPointsFirstSensor(2,:)+30*zaxes_emFirst(:,2)'],...
%     [emPointsFirstSensor(3,:); emPointsFirstSensor(3,:)+30*zaxes_emFirst(:,3)'], 'LineWidth',3,'Color', [0 0 1]);
% hold off
% 
% %% calibration
% %Sorting the positions
% 
% % [X_1_Laza, err_Laza] = handEyeLaza(Hmarker2world, H_EMT_to_EMCS)
% % we get EMT to OT here (~X_1_Laza)
% % so lets turn it around
% % X_1_Laza=inv(X_1_Laza);
% 
% % index = sortHandEyeMovement(Hmarker2world);
% % Hmarker2world = Hmarker2world(:,:,index);
% % Hgrid2cam = Hgrid2cam(:,:,index);
% % H_EMT_to_EMCS = H_EMT_to_EMCS(:,:,index);
% 
% [Hcam2marker_, err] = TSAIleastSquareCalibration(Hmarker2world, Hgrid2cam)
% % we get EMT to OT here (~Hcam2marker_)
% % so lets turn it around
% X_1 = inv(Hcam2marker_);
% 
% [X_1_Laza, err_Laza] = handEyeLaza(Hmarker2world, H_EMT_to_EMCS)
% % we get EMT to OT here (~X_1_Laza)
% % so lets turn it around
% X_1_Laza=inv(X_1_Laza);
% 
% % [X_1_dual, err_dual] = hand_eye_dual_quaternion(Hmarker2world, H_EMT_to_EMCS)
% 
% % [X_1_navy, err_navy] = navy_calibration(Hmarker2world, H_EMT_to_EMCS)
% 
% % [X_1_inria, err_inria] = inria_calibration(Hmarker2world, H_EMT_to_EMCS)
% 
% %% plot OT inside the EM-CS
% 
% %plot the transformation of OT in OCS into OT in EMCS
% opticalPoints_EMCS_transl = zeros(4,numPts);
% 
% for i=1:numPts
%     temp_EMT_to_EMCS=H_EMT_to_EMCS(:,:,i);
%     opticalPoints_EMCS_transl(:,i) = temp_EMT_to_EMCS * (X_1_Laza * [0;0;0;1]);
% end
% 
% opticalPoints_EMCS=opticalPoints_EMCS_transl(1:3,:);
% 
% figure(2)
% hold on
% plot3(opticalPoints_EMCS(1,:), opticalPoints_EMCS(2,:), opticalPoints_EMCS(3,:), 'rx', 'MarkerSize', 5)
% hold off
% 
% % show which OT and EMT should correlate
% hold on
% line([opticalPoints_EMCS(1,:); emPointsFirstSensor(1,:)],...
%     [opticalPoints_EMCS(2,:); emPointsFirstSensor(2,:)],...
%     [opticalPoints_EMCS(3,:); emPointsFirstSensor(3,:)], 'LineWidth',1,'Color', [0 0 0]);
% hold off
% 
% %make the plot nicer
% %plot cylinder which looks like our tool
% r_cylinder = 26.5/2;
% [Xcy,Ycy,Zcy] = cylinder(r_cylinder,20);
% 
% r_sphere = 4;
% [Xsp, Ysp, Zsp] = sphere;
% Xsp = Xsp * r_sphere;
% Ysp = Ysp * r_sphere;
% Zsp = Zsp * r_sphere;
% 
% H_EMT_to_EMCS_tmp = H_EMT_to_EMCS;
% 
% for i=1:numPts
% 
%     %remove offset
%     Zcy_temp = Zcy-.5;
%     %stretch to correct length
%     Zcy_temp = Zcy_temp*100; %10cm
%     
%     %rearrange EMT local cordinate system, so z points along the tool and
%     %not orthogonal to it
%     H_EMT_to_EMCS_tmp(:,:,i)=H_EMT_to_EMCS(:,[2 3 1 4],i);
%     
%     Xcy_temp = H_EMT_to_EMCS_tmp(1,1,i)*Xcy + H_EMT_to_EMCS_tmp(1,2,i)*Ycy + H_EMT_to_EMCS_tmp(1,3,i)*Zcy_temp;
%     Ycy_temp = H_EMT_to_EMCS_tmp(2,1,i)*Xcy + H_EMT_to_EMCS_tmp(2,2,i)*Ycy + H_EMT_to_EMCS_tmp(2,3,i)*Zcy_temp;
%     Zcy_temp = H_EMT_to_EMCS_tmp(3,1,i)*Xcy + H_EMT_to_EMCS_tmp(3,2,i)*Ycy + H_EMT_to_EMCS_tmp(3,3,i)*Zcy_temp;    
%        
%     %shift cylinder along axes to be positioned like our tool
%     %center at EMT position
%     Xcy_temp = Xcy_temp + emPointsFirstSensor(1,i);
%     Ycy_temp = Ycy_temp + emPointsFirstSensor(2,i);
%     Zcy_temp = Zcy_temp + emPointsFirstSensor(3,i);
%     %shift along radius, plus 3mm offset, since tool looks like that
%     Xcy_temp = Xcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(1,2,i);
%     Ycy_temp = Ycy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(2,2,i);
%     Zcy_temp = Zcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(3,2,i);
%     %shift along new z axis to represent position of EMT on tool correctly
%     Xcy_temp = Xcy_temp + 30*H_EMT_to_EMCS_tmp(1,3,i);
%     Ycy_temp = Ycy_temp + 30*H_EMT_to_EMCS_tmp(2,3,i);
%     Zcy_temp = Zcy_temp + 30*H_EMT_to_EMCS_tmp(3,3,i);
%     %shift EMT 7mm to the side, since EMT local coordinate system is not
%     %defined in the center of the tracker disc, but at the bottom of one of the
%     %screws (the right one)
%     Xcy_temp = Xcy_temp - 7*H_EMT_to_EMCS_tmp(1,1,i);
%     Ycy_temp = Ycy_temp - 7*H_EMT_to_EMCS_tmp(2,1,i);
%     Zcy_temp = Zcy_temp - 7*H_EMT_to_EMCS_tmp(3,1,i);
%     
%     %plot cylinder
%     hold on
%     surf(Xcy_temp, Ycy_temp, Zcy_temp, 'EdgeColor', 'none', 'FaceColor', 'y', 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
%     hold off
%     %plot little sphere in grey to look like the OT
%     hold on
%     surf(Xsp+opticalPoints_EMCS(1,i), Ysp+opticalPoints_EMCS(2,i), Zsp+opticalPoints_EMCS(3,i), 'EdgeColor', 'none', 'FaceColor', [.9 .9 .9], 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud')
%     hold off
%     %plot white stick that connects OT to the tool
%     hold on
%     line([opticalPoints_EMCS(1,i);opticalPoints_EMCS(1,i)+20*H_EMT_to_EMCS_tmp(1,2,i)],...
%         [opticalPoints_EMCS(2,i);opticalPoints_EMCS(2,i)+20*H_EMT_to_EMCS_tmp(2,2,i)],...
%         [opticalPoints_EMCS(3,i);opticalPoints_EMCS(3,i)+20*H_EMT_to_EMCS_tmp(3,2,i)], 'Color', 'w', 'LineWidth', 3)
%     hold off
% end
% 
% xlabel('x')
% ylabel('y')
% zlabel('z')
% axis image vis3d
% 
% set(gca,'ZDir','reverse')
% set(gca,'YDir','reverse')
% set(gca,'Color','none')
% 
% camlight('headlight')
% lighting gouraud
% 
% %% some distance statistics
%  diff=opticalPoints_EMCS-emPointsFirstSensor;
%  disp 'Distances from OT to EMT in all positions:'
% for i=1:numPts
%     norm(diff(:,i))
% end
% 
% 
% 
