function rrtDrawEdges(vertices, edges,color)
    hold on;
    grid on
    [Count,~] = size(edges);
    if size(vertices,2)>3
        error('DrawCode should be changed.current code is for 2D/3D of extended tree.')
    end
    if size(vertices,2)==3
    for i = 1 : Count
        p=(vertices(edges(i,:),:))';
        plot3(p(1,:),p(2,:),p(3,:),[color '-'])
        plot3(p(1,:),p(2,:),p(3,:),[color '*'])
    end
    elseif size(vertices,2)==2
        for i = 1 : Count
            p=(vertices(edges(i,:),:))';
            plot(p(2,:),p(1,:),[color '-'])
            plot(p(2,:),p(1,:),[color '*'])
        end
    end
end