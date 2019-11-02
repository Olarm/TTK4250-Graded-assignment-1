function qprod = quatProd(ql, qr)
    if abs(norm(ql)-1) > 0.0001
        error('Quaternion_l length not 1: %d', norm(ql));
    end
    if abs(norm(qr)-1) > 0.0001
        error('Quaternion_r length not 1: %d', norm(qr));
    end


    if numel(ql) == 3 % assume pure quat
        ql = [0; ql];
    end
    
    if numel(qr) == 3 % assume pure quat
        qr = [0; qr];
    end
    
    qprod = [ql(1)*qr(1) - ql(2:4)'*qr(2:4);
        qr(2:4)*ql(1) + ql(2:4)*qr(1) + crossProdMat(ql(2:4))*qr(2:4)];%
end