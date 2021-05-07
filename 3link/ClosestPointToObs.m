function c = ClosestPointToObs(q, C_obs)
    % This is just GJK algorithm without the beginning Minkowski
    % Difference and different inputs, since we aren't only looking at the
    % origin
    V  = C_obs(:,1:3);
    Vi = [1,2,3];
    
    % Loop
    while 1 == 1
        % Compute closest point on simplex to the origin
        if length(V) == 3
            [P, i] = ClosestPointOnTriangleToPoint(V, q);
            Q_k = Vi(i);
        elseif length(V) == 2
            B = V(:,1) - V(:,2);
            A = q - V(:,2);
            C = ((dot(A, B)/dot(B, B)) * B) + V(:,2);
            if (V(2,1) - V(2,2)) == 0
                t = (C(1)-V(1,1))/(V(1,2)-V(1,1));
            elseif (V(1,1) - V(1,2)) == 0
                t = (C(2)-V(2,1))/(V(2,2)-V(2,1));
            end
            if t < 0
                P = V(:,1);
                Q_k = Vi(1);
            elseif t > 0 && t < 1
                P = C;
                Q_k = Vi;
            elseif t > 1
                P = V(:,2);
                Q_k = Vi(2);
            end
        elseif length(V) == 1
            P = V;
            Q_k = [];
        end
        
        % Check if P is the origin
        if norm(P) <= 0
            c = P;
            break;
        end
    
        % Use the Support Mapping Function to find a further point 
        q_k = SMF(C_obs, q-P);
        
        % Check if the q_k has already been considered
        if any(ismember(q_k, Q_k))
            c = P;
            break;
        end
        
        % Define a new simplex
        Vi = [q_k, Q_k];
        V = C_obs(:, Vi);
    end
end