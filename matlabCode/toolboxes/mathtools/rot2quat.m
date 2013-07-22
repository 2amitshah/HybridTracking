% rot2quat - converts a rotation matrix (3x3) to a unit quaternion(3x1)
%
%    q = rot2quat(R)
% 
%    R - 3x3 rotation matrix, or 4x4 homogeneous matrix 
%    q - 3x1 unit quaternion
%        q = sin(theta/2) * v
%        teta - rotation angle
%        v    - unit rotation axis, |v| = 1
%
%    
% See also: quat2rot, rotx, roty, rotz, transl, rotvec

function q = rot2quat(R)

	w4 = 2*sqrt( 1 + trace(R(1:3,1:3)) ); % can this be imaginary? % Answer:
    % Not if R is a valid rotation matrix, then holds:
    % trace(R)=1+2*cos(alpha), with alpha=angle of rotation. so w4 can only
    % get 0, which will result in inf. I include an error case in the rare
    % case that this could possibly happen ;) but not zero but 10*eps, just
    % to make sure.
    % Felix Achilles, July 2013
    if (trace(R(1:3,1:3)) < (-1+10*eps))
        error('rot2quat:trace',...
            'Check if R is a valid rotation matrix. w4 is close to 0 or already imaginary, which is probably not what you want.')
    end
	q = [
		( R(3,2) - R(2,3) ) / w4;
		( R(1,3) - R(3,1) ) / w4;
        ( R(2,1) - R(1,2) ) / w4;
   ];
   
return






