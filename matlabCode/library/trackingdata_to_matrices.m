function [H_EMT_to_EMCS_cell, H_EMCS_to_EMT_cell] = trackingdata_to_matrices(EMTdata, quaternion_style)
%should be located in \library

switch quaternion_style
    case 'CppCodeQuat'
        quat_vector = 1:3;
    case 'NDIQuat'
        quat_vector = 2:4;
end

numPts = size(EMTdata,1);
numSensors = size(EMTdata,2);
mat=cell(1,numSensors);
inverse_mat = cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

for j = 1:numSensors
    for i = 1:numPts
        if ~isempty(EMTdata{i,j})
        %insert rotation into homogeneous matrix
        mat{j}(:,:,i) = quat2rot((EMTdata{i,j}.orientation(quat_vector))');
        %add translation
        mat{j}(:,:,i) = transl(EMTdata{i,j}.position') * mat{j}(:,:,i);
        
        %fill inverse matrix
        inverse_mat{j}(:,:,i) = inv(mat{j}(:,:,i));
        end
    end
end

H_EMT_to_EMCS_cell = mat;
H_EMCS_to_EMT_cell = inverse_mat;

end