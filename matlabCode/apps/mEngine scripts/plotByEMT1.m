%plot EM tracker 1
% if exist('emt1Obj', 'var'), delete(emt1Obj); end
% hold on; emt1Obj = plot3(positionEMT(1), positionEMT(2), positionEMT(3), 'x', 'Color', c(2,:) ); hold off;

% make this more secure against name confusion
orientationEMT1 = orientationEMT;
positionEMT1 = positionEMT;

% use EMT1 to map it to missing OT
% map EMT1 to OT
H_EMT1_to_EMCS = quat2rot(orientationEMT1(1:3)'); H_EMT1_to_EMCS = transl(positionEMT1') * H_EMT1_to_EMCS;
% transform EMT to OT in EMCS
H_OT_to_EMCS = H_EMT1_to_EMCS * H_OT_to_EMT;
positionOT = H_OT_to_EMCS(1:3,4);
positionOT_x = positionOT(1); positionOT_y = positionOT(2); positionOT_z = positionOT(3);
% plot OT as green circle
set(otObj,'Color','green');
refreshdata(otObj)
% plot cylinder
[~ , Xcy_temp, Ycy_temp, Zcy_temp] = Plot_cylinder(H_EMT1_to_EMCS, cylinderObj);
set(cylinderObj,'Visible','on')
set(otObj,'Visible','on')
refreshdata(cylinderObj);
set(cubeObj,'facecolor','g');
%switch off error sphere
set(redsphere, 'Visible', 'off')



drawnow
