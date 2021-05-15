function b = isintersect_linepolygon(S, Q)
    Q = [Q, Q(:,1)];
    
    % Check if S is only one point
    flag = S(:,1) == S(:,2);
    if flag(1) && flag(2)
        % Use built-in function to check if the point is in polygon
        b = inpolygon(S(1,1), S(2,1), Q(1,:), Q(2,:));
    else
        % Initialize variables
        tE = 0; tL = 1;
        ds = S(:,2) - S(:,1);
        
        for i = 1:length(Q)-1
            e = Q(:,i+1) - Q(:,i);
            % compute normal vector to edge e
            n = [e(2); -e(1)];
            if ((n(1)*e(2)-e(1)*n(2)) < 0)
                n = -n;
            end
            
            N = -dot((S(:,1)-Q(:,i)),n);
            D = dot(ds, n);
            
            if D == 0
                if N < 0
                    b = 0;
                    return;
                end
            end
            
            t = N/D;
            
            if D < 0
                tE = max(tE, t);
                if tE > tL
                    b = 0;
                    return;
                end
            elseif D > 0
                tL = min(tL, t);
                if tL < tE
                    b = 0;
                    return;
                end
            end
        end
        
        if tE <= tL
            b = 1;
            return;
        else
            b = 0;
            return;
        end
    end
end