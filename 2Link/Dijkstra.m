function path = Dijkstra(G, n_init, n_goal)
    % Set initial values
    dist = inf(1,length(G));
    prev = zeros(1,length(G));
    U = [1:length(G)];
    count = 0;
    
    % Initialize starting data
    dist(n_init) = 0;
    
    % Continue iteration while n_goal is unvisited
    while ismember(n_goal, U) && count < 10000
        
        % Iterate through U to find the node with the least distance
        Cdist = inf;
        for i = 1:length(U)
            if dist(U(i)) < Cdist
                C = U(i);
                Cdist = dist(U(i));
            end
        end
        
        % Remove C from U
        U(U == C) = []; 
        
        % Iterate through the graph adjacency matrix to find the neighbors
        % of C and update their dist with the lowest cost value
        R = G(C,:);
        for v = 1:length(R)
            alt = dist(C) + R(v);
            if alt < dist(v)
                dist(v) = alt;
                prev(v) = C;
            end
        end
        count = count + 1;
    end
    
    if count >= 10000
        path = [];
        disp('A path was not found after 5000 iterations')
        return;
    end
    
    % Reconstruct the path by iterating through the prev matrix
    path = [n_goal];
    target = n_goal;
    while target ~= n_init
        path = [prev(target), path];
        target = prev(target);
    end
end