function [path]=pathsolution(edges)
    path(1)=edges(end,1);
    while 1
        index=find(edges(:,1)==path(end));
        path=[path edges(index(1),2)];
        if path(end)==1
            break
        end
    end
end