%plot EM tracker 3
% if exist('emt1Obj', 'var'), delete(emt1Obj); end
% hold on; emt1Obj = plot3(positionEMT(1), positionEMT(2), positionEMT(3), 'x', 'Color', c(4,:) ); hold off;

% use EMT1 to map it to missing OT
% map EMT3 to EMT1
H_EMT3_to_EMCS = quat2rot(orientationEMT(1:3)'); H_EMT3_to_EMCS = transl(positionEMT') * H_EMT3_to_EMCS;
H_EMT_to_EMCS = H_EMT3_to_EMCS * H_EMT1_to_EMT3;
% transform EMT to OT in EMCS
H_OT_to_EMCS = H_EMT_to_EMCS * H_OT_to_EMT;
positionOT = H_OT_to_EMCS(1:3,4);
%plot OT as c(5,:) (lines-colormap) circle
if exist('otObj', 'var'), delete(otObj); end
hold on; otObj = plot3(positionOT(1), positionOT(2), positionOT(3), 'o', 'Color', c(5,:) ); hold off;
if exist('redsphere', 'var'), delete(redsphere); end
% plot cylinder
if exist('cylinderObj', 'var'), delete(cylinderObj); end
cylinderObj = Plot_cylinder(H_EMT_to_EMCS);