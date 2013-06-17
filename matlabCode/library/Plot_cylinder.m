function [cylinderObj, Xcy_temp, Ycy_temp, Zcy_temp] = Plot_cylinder(H_EMT_to_EMCS, cylinderObj)

%make the plot nicer
%plot cylinder which looks like our tool
r_cylinder = 26.5/2;
[Xcy,Ycy,Zcy] = cylinder(r_cylinder,20);

%remove offset
Zcy_temp = Zcy-.5;
%stretch to correct length
Zcy_temp = Zcy_temp*100; %10cm

%rearrange EMT local cordinate system, so z points along the tool and
%not orthogonal to it
H_EMT_to_EMCS_tmp(:,:,1)=H_EMT_to_EMCS(:,[2 3 1 4],1);
% H_EMT_to_EMCS_tmp(:,:,1)=H_EMT_to_EMCS(:,[3 1 2 4],1);

Xcy_temp = H_EMT_to_EMCS_tmp(1,1,1)*Xcy + H_EMT_to_EMCS_tmp(1,2,1)*Ycy + H_EMT_to_EMCS_tmp(1,3,1)*Zcy_temp;
Ycy_temp = H_EMT_to_EMCS_tmp(2,1,1)*Xcy + H_EMT_to_EMCS_tmp(2,2,1)*Ycy + H_EMT_to_EMCS_tmp(2,3,1)*Zcy_temp;
Zcy_temp = H_EMT_to_EMCS_tmp(3,1,1)*Xcy + H_EMT_to_EMCS_tmp(3,2,1)*Ycy + H_EMT_to_EMCS_tmp(3,3,1)*Zcy_temp;    

%shift cylinder along axes to be positioned like our tool
%center at EMT position
Xcy_temp = Xcy_temp + H_EMT_to_EMCS_tmp(1,4,1);
Ycy_temp = Ycy_temp + H_EMT_to_EMCS_tmp(2,4,1);
Zcy_temp = Zcy_temp + H_EMT_to_EMCS_tmp(3,4,1);
%shift along radius, plus 3mm offset, since tool looks like that
Xcy_temp = Xcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(1,2,1);
Ycy_temp = Ycy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(2,2,1);
Zcy_temp = Zcy_temp - (3+r_cylinder)*H_EMT_to_EMCS_tmp(3,2,1);
%shift along new z axis to represent position of EMT on tool correctly
Xcy_temp = Xcy_temp + 30*H_EMT_to_EMCS_tmp(1,3,1);
Ycy_temp = Ycy_temp + 30*H_EMT_to_EMCS_tmp(2,3,1);
Zcy_temp = Zcy_temp + 30*H_EMT_to_EMCS_tmp(3,3,1);
%shift EMT 7mm to the side, since EMT local coordinate system is not
%defined in the center of the tracker disc, but at the bottom of one of the
%screws (the right one)
Xcy_temp = Xcy_temp - 7*H_EMT_to_EMCS_tmp(1,1,1);
Ycy_temp = Ycy_temp - 7*H_EMT_to_EMCS_tmp(2,1,1);
Zcy_temp = Zcy_temp - 7*H_EMT_to_EMCS_tmp(3,1,1);

%plot cylinder
if ~exist('cylinderObj','var')
hold on
cylinderObj = surf(Xcy_temp, Ycy_temp, Zcy_temp, 'EdgeColor', 'none', 'FaceColor', 'y', 'FaceAlpha', 0.5, 'FaceLighting', 'gouraud');
hold off
else
% refreshdata(cylinderObj);
end
    
end