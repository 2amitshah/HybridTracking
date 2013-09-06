function L_infinity_norm = convex_obj_fcn(x,C,d)
    N=numel(C);
    err=zeros(1,N);
    for i=1:N
        err(i)=norm(C{i}*x-d{i});
    end
    L_infinity_norm=max(err);
end