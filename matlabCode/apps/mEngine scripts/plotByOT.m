% create H matrix for optical
H_OT_to_OCS = quat2rot(orientationOT_OCS(1:3)'); H_OT_to_OCS = transl(positionOT_OCS') * H_OT_to_OCS;
% transform to EMCS
H_OT_to_EMCS = Y * H_OT_to_OCS;
positionOT = H_OT_to_EMCS(1:3,4);
positionOT_x = positionOT(1); positionOT_y = positionOT(2); positionOT_z = positionOT(3);
% plot OT as blue circle
set(otObj,'Color','blue');
refreshdata(otObj)
% plot tool as yellow cylinder
H_EMT1_to_EMCS = H_OT_to_EMCS/H_OT_to_EMT;
[~ , Xcy_temp, Ycy_temp, Zcy_temp] = Plot_cylinder(H_EMT1_to_EMCS, cylinderObj);
refreshdata(cylinderObj);
set(cubeObj,'facecolor','b');
%switch off error sphere
set(redsphere, 'Visible', 'off')
%switch on cylinder and tracker dot
set(cylinderObj,'Visible','on')
set(otObj,'Visible','on')



drawnow
