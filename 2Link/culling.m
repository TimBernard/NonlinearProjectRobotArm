function [path_cull, indx_cull] = culling(path, path_indx, O)
    P1 = path(:,1);
    path_cull = P1;
    indx_cull = 1;
    last = 0;
    for i = 2:length(path)-1
        if ~isnocollision([P1,path(:,i)], O) || last > 5
            path_cull = [path_cull, path(:,i-1)];
            indx_cull = [indx_cull, path_indx(i-1)];
            P1 = path(:,i);
            last = 0;
        else
            last = last + 1;
        end
    end
    path_cull = [path_cull, path(:,end)];
    indx_cull = [indx_cull, path_indx(end)];
end