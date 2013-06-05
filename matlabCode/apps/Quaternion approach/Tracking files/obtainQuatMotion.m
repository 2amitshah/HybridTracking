function [ quatRot, quatTr ] = obtainQuatMotion( H1, H2 )

    H1to2= inv(H2)*H1;
    
    quatTr = H1to2(1:3,4);
    quatRot = rot2quat(H1to2(1:3,1:3));
    
end

