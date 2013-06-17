%plot EM tracker 2
% if exist('emt1Obj', 'var'), clear emt1Obj; end
% hold on; emt1Obj = plot3(positionEMT(1), positionEMT(2), positionEMT(3), 'x', 'Color', c(3,:) ); hold off;

% use EMT1 to map it to missing OT
% map EMT2 to EMT1
H_EMT2_to_EMCS = quat2rot(orientationEMT(1:3)'); H_EMT2_to_EMCS = transl(positionEMT') * H_EMT2_to_EMCS;
H_EMT_to_EMCS = H_EMT2_to_EMCS * H_EMT1_to_EMT2;
% transform EMT to OT in EMCS
H_OT_to_EMCS = H_EMT_to_EMCS * H_OT_to_EMT;
positionOT = H_OT_to_EMCS(1:3,4);
c_objects = findobj;
%plot OT as red circle
if any(c_objects == otObj)
    delete(otObj);
end
hold on; otObj = plot3(positionOT(1), positionOT(2), positionOT(3), 'o', 'Color', c(3,:) ); hold off;
if any(c_objects == redsphere)
    delete(redsphere);
end
% plot cylinder
if any(c_objects == cylinderObj)
    delete(cylinderObj);
end
cylinderObj = Plot_cylinder(H_EMT_to_EMCS);