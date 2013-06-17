% create H matrix for optical
H_OT_to_OCS = quat2rot(orientationOT_OCS(1:3)'); H_OT_to_OCS = transl(positionOT_OCS') * H_OT_to_OCS;
% transform to EMCS
H_OT_to_EMCS = Y * H_OT_to_OCS;
positionOT = H_OT_to_EMCS(1:3,4);
c_objects = findobj;
% plot red sphere on last known OT position
xsp = 25*x + positionOT(1); ysp = 25*y + positionOT(2); zsp = 25*z + positionOT(3);
if any(c_objects == redsphere)
    delete(redsphere);
end
hold on; redsphere = surf(xsp,ysp,zsp, 'EdgeColor' , 'none', 'FaceColor', 'r', 'FaceLighting', 'gouraud'); hold off; 