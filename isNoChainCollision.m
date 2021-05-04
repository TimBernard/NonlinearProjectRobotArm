function b = isNoChainCollision(x, O)
    for i = 1:length(O)
        for j = 1:length(x)-1
            if isintersect_linepolygon([x(:,j), x(:,j+1)], O{i})
                b = 0;
                return;
            end
        end
    end
    
    % Check to see if S intersects with any O
    b = 1;
end