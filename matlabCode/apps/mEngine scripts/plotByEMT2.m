%plot EM tracker 2
% if exist('emt1Obj', 'var'), clear emt1Obj; end
% hold on; emt1Obj = plot3(positionEMT(1), positionEMT(2), positionEMT(3), 'x', 'Color', c(3,:) ); hold off;

% make this more secure against name confusion
orientationEMT2 = orientationEMT;
positionEMT2 = positionEMT;

% use EMT1 to map it to missing OT
% map EMT2 to EMT1
H_EMT2_to_EMCS = quat2rot(orientationEMT2(1:3)'); H_EMT2_to_EMCS = transl(positionEMT2') * H_EMT2_to_EMCS;
H_EMT1_to_EMCS = H_EMT2_to_EMCS * H_EMT1_to_EMT2;
% transform EMT to OT in EMCS
H_OT_to_EMCS = H_EMT1_to_EMCS * H_OT_to_EMT;
positionOT = H_OT_to_EMCS(1:3,4);
positionOT_x = positionOT(1); positionOT_y = positionOT(2); positionOT_z = positionOT(3);
% plot OT as yellow circle
set(otObj,'Color','y');
refreshdata(otObj)
% plot cylinder
[~ , Xcy_temp, Ycy_temp, Zcy_temp] = Plot_cylinder(H_EMT1_to_EMCS, cylinderObj);
set(cylinderObj,'Visible','on')
set(otObj,'Visible','on')
refreshdata(cylinderObj);
set(cubeObj,'facecolor','y');
%switch off error sphere
set(redsphere, 'Visible', 'off')



drawnow
