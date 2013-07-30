function [weights] = weightFcn(indices, frames)
speeds = zeros(size(indices)); % unit: mm/s
for i = 2:numel(indices)
    speeds(i) = norm(frames(1:3,4,i-1)-frames(1:3,4,i))*20; % 20Hz
end
speeds(speeds < 5)=0; % does not move
speeds(speeds >= 5)=1; % does move
weights =speeds;
end