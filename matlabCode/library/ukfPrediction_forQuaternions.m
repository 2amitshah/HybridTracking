function [x,P]=ukfPrediction_forQuaternions(fstate,x,P,Q)
% UKF   Unscented Kalman Filter for nonlinear dynamic systems
% [x, P] = ukf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P 
% for nonlinear dynamic system (for simplicity, noises are assumed as additive):
%           x_k+1 = f(x_k) + w_k
%           z_k   = h(x_k) + v_k
%
% special version for quaternions: Noise must be assumed to be
% multiplicative for the quaternion (parts x(7:10))
%
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   f: function handle for f(x)
%           x: "a priori" state estimate
%           P: "a priori" estimated state covariance
%           h: fanction handle for h(x)
%           z: current measurement
%           Q: process noise covariance 
%           R: measurement noise covariance
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance
%           residual: difference of estimation and measurement
%
% Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
% Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3,
% pp.401-422, 2004. 
%
% By Yi Cao at Cranfield University, 04/01/2008
%
if numel(x) ~= 13
    error('x is not a correct state vector [p,p_dot,q,w]''')
end

L=numel(x);                                 %number of states
alpha=1e-3;                                 %default, tunable
ki=0;                                       %default, tunable
beta=2;                                     %default, tunable
lambda=alpha^2*(L+ki)-L;                    %scaling factor
c=L+lambda;                                 %scaling factor
Wm=[lambda/c 0.5/c+zeros(1,2*L)];           %weights for means
Wc=Wm;
Wc(1)=Wc(1)+(1-alpha^2+beta);               %weights for covariance
c=sqrt(c);

X=sigmas_forQuaternions(x,P,c);             %sigma points around x
[x1,~,P1]=ut_forQuaternions(fstate,X,Wm,Wc,L,Q);       %unscented transformation of process
% X1=sigmas(x1,P1,c);                         %sigma points around x1
% X2=X1-x1(:,ones(1,size(X1,2)));             %deviation of X1
x=x1;                                       %state prediction
P=P1;                                       %covariance prediction
end

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
qP = qP/quatnorm(qP');
qP = quatmultiply(diag(R(7:10,7:10))',qP')'; %rotate qP about qR
P(7:10,7:10)=diag(qP.^2);
end

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
% % A(7:10,i) = (quatnormalize(A(7:10,i)'))';
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


