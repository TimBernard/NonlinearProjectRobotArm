function [path, V, G] = build_PRM(q_I, q_G, n, K, O, xmax, ymax)
    % Initialize Variables
    V = [];
    G = Inf * ones(n);
    for i = 1:length(G)
        G(i,i) = 0;
    end
    
    % Generate random points q in C_free until V is full
    while length(V) < n
        q_rand = [xmax*rand(); ymax*rand()];
        % Check to see if q_rand is not in O
        if iscollisionfree([q_rand, q_rand], O)
            V = [V, q_rand];
        end
    end
    
    for i = 1:length(V)
        % Find the K-nearest neighbors for each point in V
        [N, indx] = knn(V(:,i), V, K+1);
        for j = 1:length(N)
            % Check if edge already exists in the weighted adjacency matrix
            % G and that the distance isn't 0. Also check that the path
            % doesn't collide with any objects in O
            if G(i,indx(j)) == Inf && N(j) ~= 0 && iscollisionfree([V(:,i), V(:,indx(j))],O)
                G(i, indx(j)) = N(j);
                G(indx(j), i) = N(j);
            end
        end
    end
    
    % Use Dijkstra to find the path
    [v_1, q_1] = knn(q_I, V, length(V));
    [v_end, q_end] = knn(q_G, V, length(V));
    path_indx = Dijkstra(G, q_1(1), q_end(1));
    path = V(:,path_indx);
end

% K-nearest neighbor search algorithm
function [D, indx] = knn(P, V, k)
    % Calculate distance between all points and note their index
    N = [zeros(1,length(V)); 1:length(V)];
    for i = 1:length(V)
        N(1,i) = norm(P-V(:,i));
    end
    % Sort the distances in ascending order
    N = sortrows(N')';
    % Return the first k elements
    D = N(1,1:k);
    indx = N(2,1:k);
end

% Checks for collision with any member of set O
% Returns true if there are no collisions
function b = iscollisionfree(S, O)
    if ~isempty(O)
        flag = 1:length(O);
        for i = 1:length(O)
            flag(i) = isintersect_linepolygon(S, O{i});
        end
    % Check to see if S intersects with any O
    b = all(~flag);
    else
        b = 1;
    end
end