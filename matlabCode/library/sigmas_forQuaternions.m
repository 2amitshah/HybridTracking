function X=sigmas_forQuaternions(x,P,c)
%Sigma points around reference point
%Inputs:
%       x: reference point
%       P: covariance
%       c: coefficient
%Output:
%       X: Sigma points
L=numel(x); 
A = c*chol(P)';
% re-normalize the quaternions in A
% for i = 1:L
%     A(7,i) = min(1,A(7,i)); % make it a quaternion haha
%     A(7,i) = max(0,A(7,i)); % make it a quaternion haha
%     A(7:10,i) = A(7:10,i)/(quatnorm(A(7:10,i)')+0.001);
% %     A(7:10,i) = (quatnormalize(A(7:10,i)'))';
% end
Y = x(:,ones(1,L)); % x is repeated L times as a column
X = [x Y+A Y-A];           % parts x(7:10) need to be multiplied, not added
% for i =1:2*L+1
%     if i==1
%         X(7:10,i) = x(7:10);
%     elseif i<=(L+1)
%         X(7:10,i) = quatmultiply(A(7:10,i-1)' ,x(7:10)'); %rotate qx by qA
%     elseif i<=(2*L+1)
%         X(7:10,i) = quatmultiply(quatinv(A(7:10,i-1-L)') ,x(7:10)'); %rotate qx by inverse of qA
%     end
% end

end