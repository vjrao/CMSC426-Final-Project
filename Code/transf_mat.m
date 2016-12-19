function M = transf_mat(n, d, s)
    b = sum(n.*d,2)-sum(n.*s,2);
    a1 = n(:,3).*s(:,2)-n(:,2).*s(:,3);
    a2 = n(:,1).*s(:,3)-n(:,3).*s(:,1);
    a3 = n(:,2).*s(:,1)-n(:,1).*s(:,2);
    A = [a1,a2,a3,n(:,1),n(:,2),n(:,3)];
    [U,S,V] = svd(A,'econ');
    S_new = 1./S;
    S_new(S_new==inf) = 0;
    %A_old = U*S*V';
    A_new = V*S_new*U';
    x_opt = A_new*b;
    R = [1,x_opt(1)*x_opt(2)-x_opt(3),x_opt(1)*x_opt(3)+x_opt(2),0;
        x_opt(3),x_opt(1)*x_opt(2)*x_opt(3)+1,x_opt(2)*x_opt(3)-x_opt(1),0;
        -x_opt(2),x_opt(1),1,0;
        0,0,0,1];
    T = eye(4);
    T(:,4) = [x_opt(4);x_opt(5);x_opt(6);1];
    M = T*R;
end
