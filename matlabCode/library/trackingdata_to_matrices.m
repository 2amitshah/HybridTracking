function [H_X_to_XBase_cell, H_XBase_to_X_cell] = trackingdata_to_matrices(Xdata, quaternion_style)
%should be located in \library

switch quaternion_style
    case 'CppCodeQuat'
        quat_vector = 1:3;
    case 'NDIQuat'
        quat_vector = 2:4;
end

validexists = false;
if isfield(Xdata, 'valid')
    validexists = true;
end

numPts = size(Xdata,1);
numSensors = size(Xdata,2);
mat=cell(1,numSensors);
inverse_mat = cell(1,numSensors);

for j = 1:numSensors
    mat{j} = zeros(4, 4, numPts);
end

for j = 1:numSensors
    for i = 1:numPts
        if ~isempty(Xdata{i,j})
            if validexists
                if Xdata{i,j}.valid
                    %insert rotation into homogeneous matrix
                    mat{j}(:,:,i) = quat2rot((Xdata{i,j}.orientation(quat_vector))');
                    %add translation
                    mat{j}(:,:,i) = transl(Xdata{i,j}.position') * mat{j}(:,:,i);

                    %fill inverse matrix
                    inverse_mat{j}(:,:,i) = inv(mat{j}(:,:,i));
                end
            else
                %insert rotation into homogeneous matrix
                mat{j}(:,:,i) = quat2rot((Xdata{i,j}.orientation(quat_vector))');
                %add translation
                mat{j}(:,:,i) = transl(Xdata{i,j}.position') * mat{j}(:,:,i);

                %fill inverse matrix
                inverse_mat{j}(:,:,i) = inv(mat{j}(:,:,i));
            end
        end
    end
end

H_X_to_XBase_cell = mat;
H_XBase_to_X_cell = inverse_mat;

end