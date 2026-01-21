function tau = rep(q,myrobot,obs)
    eta = 1;
    forces = zeros(3,length(q));
    tau = zeros(1,length(q));
    
    o = zeros(3, length(q)+1);
    z = zeros(3, length(q)+1);
    z(1:3,1) = [0 0 1]';

    for i=1:length(q)
        J = zeros(3, length(q));

        % Compute forward dyanmics
        H_actual = forward(q, myrobot, i);
        o_actual = H_actual(1:3, 4);

        if obs.type == 'sph'
            distance = o_actual - ((obs.R .* (o_actual - obs.c) ./ norm(o_actual-obs.c)) + obs.c);
        elseif obs.type == 'cyl'
            % same as inf cylinder case
            if o_actual(3) <= obs.h
                temp = [o_actual(1)-obs.c(1) o_actual(2)-obs.c(2) 0]';
                distance = o_actual - ((obs.R .*(temp) / norm(temp)) + [obs.c(1) obs.c(2) o_actual(3)]');
            % if above cylinder
            else
                %right on top of cylinder
                if norm(o_actual(1:2) - obs.c) <= obs.R
                    distance = [0 0 o_actual(3)-obs.h];
                else %above and to the side, project onto rim of cylinder
                    pi = obs.c + obs.R .* ((o_actual(1:2) - obs.c) / norm(o_actual(1:2) - obs.c));
                    distance = o_actual - [pi(1) pi(2) obs.h]';
                end
            end

        else %if obs.type == 'pla'
            distance = [0 0 o_actual(3)-obs.h];
        end

        norm_dist = norm(distance);
        grad_rho = distance/norm_dist;
        
        if norm_dist <= obs.rho0
            forces(:,i) = eta .* (((1/norm_dist) - (1/obs.rho0)) .* (grad_rho./norm_dist^2));
        end

        o(:,i+1) = o_actual;
        z(:,i+1) = H_actual(1:3, 3);
       
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