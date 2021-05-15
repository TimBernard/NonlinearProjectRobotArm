function p = SMF(obj, v)
    h = zeros(1,length(obj));
    for i = 1:length(obj)
        h(i) = dot(v, obj(:,i));
    end
    m = max(h);
    [~, p] = find(h==m);
end