% fstate_OT=@(x)(x + [currentTimestep * x(4); currentTimestep * x(5); currentTimestep * x(6); zeros(statesize-3,1)])
function [x_minus] = transitionFunction_f(x, currentTimestep)
if numel(x) ~= 13
    error('x is not a correct state vector [p,p_dot,q,w]''')
end

q = x(7:10)';
w = x(11:13);

eulervec = w*currentTimestep;

dq = angle2quat(eulervec(1), eulervec(2), eulervec(3), 'XYZ');
q_minus = quatmultiply(q, dq);

x_minus = [ x(1) + currentTimestep * x(4);
            x(2) + currentTimestep * x(5);
            x(3) + currentTimestep * x(6);
            x(4:6);
            q_minus';
            w ];
end