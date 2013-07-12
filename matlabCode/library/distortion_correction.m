function data_EM_common_corrected = distortion_correction(data_EM_common)

%% use Fu,Fv and Fw (distortion map) to compute better EM positions
load('Fu');
load('Fv');
load('Fw');
amountNan = 0;
for j = 1:size(data_EM_common,2)
for i = 1:size(data_EM_common,1)
    if data_EM_common{i,j}.valid
       oldX = data_EM_common{i,j}.position(1);
       oldY = data_EM_common{i,j}.position(2);
       oldZ = data_EM_common{i,j}.position(3);
       if ~isnan(Fu(oldX, oldY, oldZ))
           data_EM_common{i,j}.position(1) = data_EM_common{i,j}.position(1) - Fu(oldX, oldY, oldZ);
       else
           i                   
           amountNan = amountNan+1;
       end
       if ~isnan(Fv(oldX, oldY, oldZ))
           data_EM_common{i,j}.position(2) = data_EM_common{i,j}.position(2) - Fv(oldX, oldY, oldZ);
       else
           i                   
           amountNan = amountNan+1;
       end
       if ~isnan(Fw(oldX, oldY, oldZ))
           data_EM_common{i,j}.position(3) = data_EM_common{i,j}.position(3) - Fw(oldX, oldY, oldZ);
       else
           i                   
           amountNan = amountNan+1;
       end
    end
end
end
amountNan
data_EM_common_corrected = data_EM_common;

end