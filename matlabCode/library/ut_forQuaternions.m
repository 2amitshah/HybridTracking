function [y,Y,P,Y1]=ut_forQuaternions(f,X,Wm,Wc,n,R)
%Unscented Transformation
%Input:
%        f: nonlinear map
%        X: sigma points
%       Wm: weights for mean
%       Wc: weights for covraiance
%        n: numer of outputs of f
%        R: additive covariance
%Output:
%        y: transformed mean
%        Y: transformed smapling points
%        P: transformed covariance
%       Y1: transformed deviations

L=size(X,2);
y=zeros(n,1);
y(7:10) = [1;0;0;0];
Y=zeros(n,L);
for k=1:L                   
    Y(:,k)=f(X(:,k));
    yquat = y(7:10);
    y=y+Wm(k)*Y(:,k);  %wroooong, parts x(7:10) need to be multiplied, not added  
    % if weight W(k) is high, you go a lot in Y(7:10)-direction, if not, you stick with old y
    y(7:10) = (slerp(yquat', Y(7:10,k)', Wm(k),eps))';
end
Y1=Y-y(:,ones(1,L)); % the difference between weighted mean prediction and single predictions
% lets repeat the quat part... Y1=qmult(Y,qinv(y))
Y1(7:10,:)=(quatmultiply( Y(7:10,:)' ,quatinv(y(7:10,:)') ))';
% now how do we do this... it is strange to multiply the quaternion with
% the rest of the values... but hey let's try and at least add R correctly
R_temp = R;
R_temp(7,7) = 0;
R_temp(8,8) = 0;
R_temp(9,9) = 0;
R_temp(10,10) = 0;
P=Y1*diag(Wc)*Y1'+R_temp;
qP = diag(P(7:10,7:10));
qP(1) = min(1,qP(1));
qP(1) = max(0,qP(1));
qP = qP/(quatnorm(qP')+0.001);
qP = quatmultiply(diag(R(7:10,7:10))',qP')'; %rotate qP about qR
P(7:10,7:10)=diag(qP.^2);
end
