function rrtDrawPath(vertices, path,color)
    hold on;
    grid on
    [~, pathCount] = size(path);
    if size(vertices,2)>3
        error('DrawCode should be changed.current code is for 2D/3D of extended tree.')
    end
    if size(vertices,2)==3
        for i = 1 : pathCount-1
            p=(vertices(path(i:i+1),:))';
            plot3(p(1,:),p(2,:),p(3,:),[color '-'])
            plot3(p(1,:),p(2,:),p(3,:),[color '*'])
        end
    elseif size(vertices,2)==2
        for i = 1 : pathCount-1
            p=(vertices(path(i:i+1),:))';
            plot(p(2,:),p(1,:),[color '-'])
            plot(p(2,:),p(1,:),[color '*'])
        end
    end
    
end