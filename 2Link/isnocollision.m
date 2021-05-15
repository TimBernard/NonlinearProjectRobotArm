% Checks for collision with any member of set O
% Returns true if there are no collisions
function b = isnocollision(S, O)
    for i = 1:length(O)
        if isintersect_linepolygon(S, O{i})
            b = 0;
            return;
        end
    end
    
    b = 1;
end