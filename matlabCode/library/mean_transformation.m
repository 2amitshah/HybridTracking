function [H_avg] = mean_transformation(H_data)
% mean_transformation correctly computes the mean translation and rotation
% 4x4 matrix from H_data(4x4xNumPoints).

N = size(H_data,3);
sum_of_log = zeros(4);
for i = 1:N
    sum_of_log = sum_of_log + logm(H_data(:,:,i));
end

H_avg = expm(sum_of_log/N);
% to get nice homogenuous matrices, hard-code the last row
H_avg(4,:) = [0 0 0 1];

end