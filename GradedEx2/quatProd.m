function qprod = quatProd(ql, qr)

    if numel(ql) == 3 % assume pure quat
        ql = [0; ql];
    end
    
    if numel(qr) == 3 % assume pure quat
        qr = [0; qr];
    end
    
    qprod = [ql(1)*qr(1) + ql(2:4)'*qr(2:4);
        qr(2:4)*ql(1) + ql(2:4)*qr(1) + crossProdMat(ql(2:4))*qr(2:4)];%
end