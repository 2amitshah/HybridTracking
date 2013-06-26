%%%%%%%%%%%%%%%%
%
% trackingdata_to_matrices takes a Xdata cellstruct with the fields .position,
% .orientation and .valid and creates homogenuous (H-) matrices in the cells
% H_X_to_XBase_cell and H_XBase_to_X_cell.
%
% If some cells in the input are empty or invalid, the frames will be 4x4
% zeros.


function [H_X_to_XBase_cell, H_XBase_to_X_cell] = trackingdata_to_matrices(Xdata, quaternion_style)
%should be located in \library

switch quaternion_style
    case 'CppCodeQuat'
        quat_vector = 1:3;
    case 'NDIQuat'
        quat_vector = 2:4;
end

numPts = size(Xdata,1);
numSensors = size(Xdata,2);

validexists = false;
i = 1;
while(i <= numPts && validexists == false)
    if ~isempty(Xdata{i,1})
        if isfield(Xdata{i,1}, 'valid')
            validexists = true;
        end
    end
    i = i + 1;
end

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
                else
%                     disp 'empty or invalid struct entry'
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