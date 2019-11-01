function R = quat2rotmat(quat)  
    eta = quat(1);
    e = quat(2:4);          % n
    S = crossProdMat(e);
    
    I = eye(3);
    
    % cos(a)I + (1 - cos(a))nn' + sin(a)S(n)
    R = cos(eta)*I + (1 - cos(eta))*(e*e') + sin(eta)*S;
end