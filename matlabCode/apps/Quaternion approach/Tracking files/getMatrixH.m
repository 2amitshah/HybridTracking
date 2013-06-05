function H = getMatrixH( r,t )
    rotMatrix = quat2rot(r);
    H = [rotMatrix(1:3,1:3) t;
        zeros(1,3) 1];
end

