% create H matrix for optical
% H_OT_to_OCS = quat2rot(orientationOT_OCS(1:3)'); H_OT_to_OCS = transl(positionOT_OCS') * H_OT_to_OCS;
% transform to EMCS
% H_OT_to_EMCS = Y * H_OT_to_OCS;
% positionOT = H_OT_to_EMCS(1:3,4);
% plot red sphere on last known OT position
% xsp = 25*x + positionOT(1); ysp = 25*y + positionOT(2); zsp = 25*z + positionOT(3);
xsp = 25*x + positionOT_x; ysp = 25*y + positionOT_y; zsp = 25*z + positionOT_z;
refreshdata(redsphere)
set(redsphere, 'Visible', 'on')
set(cubeObj,'facecolor','r')
set(cylinderObj,'Visible','off')
set(otObj,'Visible','off')
%update textbox
set(textboxObj,'Position',[positionOT_x+10 positionOT_y+10 positionOT_z-40],'String','NO Tracker available!')

drawnow


