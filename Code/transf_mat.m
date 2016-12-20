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
    al = x_opt(1);
    bet = x_opt(2);
    gam = x_opt(3);


    R = [cos(gam)*cos(bet),-sin(gam)*cos(al)+cos(gam)*sin(bet)*sin(al),sin(gam)*sin(al)+cos(gam)*sin(bet)*cos(al),0;
        sin(gam)*cos(bet),cos(gam)*cos(al)+sin(gam)*sin(bet)*sin(al),-cos(gam)*sin(al)+sin(gam)*sin(bet)*cos(al),0;
        -sin(bet),cos(bet)*sin(al), cos(bet)*cos(al),0;
        0,0,0,1];
    T = eye(4);
    T(:,4) = [x_opt(4);x_opt(5);x_opt(6);1];
    M = T*R;
end
