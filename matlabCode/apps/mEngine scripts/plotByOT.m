% create H matrix for optical
H_OT_to_OCS = quat2rot(orientationOT_OCS(1:3)'); H_OT_to_OCS = transl(positionOT_OCS') * H_OT_to_OCS;
% transform to EMCS
H_OT_to_EMCS = Y * H_OT_to_OCS;
positionOT = H_OT_to_EMCS(1:3,4);
positionOT_x = positionOT(1);
positionOT_y = positionOT(2);
positionOT_z = positionOT(3);
% c_objects = findobj;

%plot OT as red circle
refreshdata(otObj)
% if any(c_objects == otObj)
%     delete(otObj);
% end
% hold on; otObj = plot3(positionOT(1), positionOT(2), positionOT(3), 'o', 'Color', c(3,:) ); hold off;
% if any(c_objects == redsphere)
%     delete(redsphere);
% end

H_EMT_to_EMCS = H_OT_to_EMCS/H_OT_to_EMT;

% plot cylinder
% if any(c_objects == cylinderObj)
%     delete(cylinderObj);
% end
[~ , Xcy_temp, Ycy_temp, Zcy_temp] = Plot_cylinder(H_EMT_to_EMCS, cylinderObj);
refreshdata(cylinderObj);