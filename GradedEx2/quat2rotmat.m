function R = quat2rotmat(quat) 
    if abs(norm(quat)-1) > 0.0001
        error('Quaternion length not 1: %d', norm(quat));
    end
    eta = quat(1);
    e = quat(2:4);          % n
    S = crossProdMat(e);
    
    I = eye(3);
    
    % cos(a)I + (1 - cos(a))nn' + sin(a)S(n)
    %R = cos(eta)*I + (1 - cos(eta))*(e*e') + sin(eta)*S;
    R = I + 2*eta*S + 2*S*S;
end