function interp_quaternions = quatInterp(timestamps_original_vector, quaternions, timestampsNewVector, quaternionStyle)

if ~exist('quaternionStyle','var')
    error('quatinterp::inputs - you forgot quaternionStyle')
end
switch quaternionStyle
    case 'cpp'
        quat_vector = 1:3;
        quaternions = quaternions(:,[4 quat_vector]);
    case 'ndi'
        quat_vector = 2:4;
end
numTS = numel(timestampsNewVector);
interp_quaternions = zeros(numTS, 4);

for i = 1:numTS
    t = timestampsNewVector(i);
    % find preceeding and successing timestamp
    pi_ind = find(timestamps_original_vector <= t,1,'last');
    pn_ind = find(timestamps_original_vector > t,1,'first');
    if isempty(pi_ind) || isempty(pn_ind)
        error('quatinterp::findInterval - Synthetic timestamp not inbetween two original timestamps!');
    end
    % calculate delta TS and the distance ratio r
    delta_TS = timestamps_original_vector(pn_ind) - timestamps_original_vector(pi_ind);
    if delta_TS == 0
        error('quatinterp::findInterval - two succeding original timestamps are equal!')
    end
    % r between 0 and 1
    r = (t - timestamps_original_vector(pi_ind)) / delta_TS;
    % perform interpolation
    qi = quaternions(pi_ind,:);
    qi = qi / norm(qi);
    qn = quaternions(pn_ind,:);
    qn = qn / norm(qn);
    qm = slerp(qi, qn, r, eps);
    interp_quaternions(i,:) = qm;
end



end