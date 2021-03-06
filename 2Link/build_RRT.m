function [path_indx, V, E, G, Q] = build_RRT(q_I, x_G, n, dx, O, xmax, ymax)
    % Initialize Variables
    Q = q_I;
    x_I = fwdKin(q_I);
    V = x_I(:,end);
    E = [];
    G = Inf * ones(n);
    for i = 1:length(G)
        G(i,i) = 0;
    end
    
    for i = 1:n
        % Continuously generate x_rand until it is does not collide with
        % any obstacle
        flag = 1;
        while flag
            x_rand = [xmax*rand(); ymax*rand()];
            q_rand = invKin(x_rand);
            if isnocollision([x_rand, x_rand], O) && isNoChainCollision(fwdKin(q_rand), O)
                flag = 0;
            end
        end
        
        % Find the nearest point to x_rand that is already in the tree
        [x_near, i_near] = Nearest_Vertex(x_rand, V);
        
        % Move dx along the direction from q_near to q_rand or just use
        % x_rand if it is smaller than dx
        if norm(x_rand - x_near) > dx
            x_new = x_near + (x_rand - x_near) * (dx / norm(x_rand - x_near));
            q_new = invKin(x_new);
        else
            x_new = x_rand;
            q_new = q_rand;
        end
        
        % Check if x_new will collide with O or if its out of bounds
        if isnocollision([x_near, x_new], O) && max(x_new) < max(xmax,ymax) && min(x_new) > 0 && isNoChainCollision(fwdKin(q_new), O)
            % Add new values to graph
            V = [V, x_new(:,end)];
            Q = [Q, q_new];
            E = [E, [i_near; length(V)]];
            G(i_near, length(V)) = norm(x_new - x_near);
            G(length(V), i_near) = norm(x_new - x_near);
        end
    end
    
    % Find the points in V that are the closest to x_I and x_G
    [~, x_1] = Nearest_Vertex(x_I(:,end), V);
    [~, x_end] = Nearest_Vertex(x_G, V);
    
    % Find shortest path using Dijkstra's algorithm
    path_indx = Dijkstra(G, x_1, x_end);
end

% Returns the nearest vertex in V from q
function [v_new, indx] = Nearest_Vertex(q, V)
    d = inf; v_new = [];
    s = size(V);
    for i = 1:s(2)
        % If new distance is smaller, update values
        if norm(V(:,i)-q) < d
            v_new = V(:,i);
            indx = i;
            d = norm(V(:,i)-q);
        end
    end
end