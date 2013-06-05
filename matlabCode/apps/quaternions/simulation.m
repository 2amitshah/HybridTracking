function result = simulation( x,eqsystem )
syms rx ry rz tx ty tz;

% a = subs(eqsystem,qx,x(1));
% b = subs(a,qy,x(2));
% c = subs(b,tx,x(4));
% d = subs(c,ty,x(5));
% e = subs(d,tz,x(6));
a = max(abs(double(subs(eqsystem, [rx; ry; rz; tx; ty; tz], x))));
result = a;
end

