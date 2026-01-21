function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    alpha_i = 0.02;
    N = 2000;
    q = q0;

    for i = 1:N-1
        all_rep = 0;

        for j = 1:length(obs)
            all_rep = all_rep + rep(q(i,:),myrobot,obs{j});
        end

        q_next = q(i, :) + alpha_i.*((att(q(i,:),q2,myrobot))+all_rep);
        q = [q; q_next];

        if norm(q(end,1:5)-q2(1:5))<tol
            break;
        end
    end

    q(:,6) = linspace(q0(6),q2(6),size(q,1))';

    t = linspace(t1,t2,size(q,1));

    qref = spline(t,q');
end