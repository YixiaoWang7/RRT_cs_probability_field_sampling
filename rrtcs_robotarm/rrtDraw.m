function rrtDraw(space, vertices, path,BasePoint,angle)
    hold on;
    grid on
    [~, pathCount] = size(path);
    for i = 1 : pathCount
        theta=vertices(path(i),:);
        T=fkine_UR10(theta,0);
        [SetPointS]=Joints0(BasePoint,angle,T);
        h=draw_arm(SetPointS);
        axis equal
        axis([space(:,1)' space(:,2)' space(:,3)'])
        pause(0.2)
        for m=1:9
            %delete(h(m));
        end
    end
end