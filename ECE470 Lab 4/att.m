function tau = att(q,q2,myrobot)
    zeta = ones(1,length(q));
    forces = zeros(3,length(q));
    tau = zeros(1,length(q));
    
    o = zeros(3, length(q)+1);
    z = zeros(3, length(q)+1);
    z(1:3,1) = [0 0 1]';

    for i=1:length(q)
        J = zeros(3, length(q));

        % Compute forward dyanmics
        H_actual = forward(q, myrobot, i);
        H_final = forward(q2, myrobot, i);
    
        o_actual = H_actual(1:3, 4);
        o_final = H_final(1:3, 4);

        o(:,i+1) = o_actual;
        z(:,i+1) = H_actual(1:3, 3);
       
        % Compute artifical forces
        forces(:,i) = -zeta(i) .* (o_actual - o_final).* 100;

        % Compute J_vis
        for j = 1:i
            J(:,j) = cross(z(1:3,j), (o_actual-o(:,j)));
        end

        % Compute tau
        tau = tau + (J' * forces(:,i))';
    end

    if norm(tau) ~= 0
        tau = tau/norm(tau);
    end
end