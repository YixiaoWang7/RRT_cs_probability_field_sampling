function rrtDrawA(space, vertices, path,BasePoint,angle)
    title('Path planning of catching M18(blue) and M22(rad) gaskets');
    hold on;
    grid on
    l=1;
    for i = 1 : size(vertices,1)
        if l<=length(path)
        if i==path(l)
            l=l+1;
            theta=vertices(i,:);
            T=fkine_UR10(theta,0);
            [SetPointS]=Joints0(BasePoint,angle,T);
            h=draw_armA(SetPointS,'r');
            axis equal
            axis([space(:,1)' space(:,2)' space(:,3)'])
            pause(0.2)
            for m=1:9
                %delete(h(m));
            end
        else
            theta=vertices(i,:);
            T=fkine_UR10(theta,0);
            [SetPointS]=Joints0(BasePoint,angle,T);
            h=draw_armA(SetPointS,'g');
            axis equal
            axis([space(:,1)' space(:,2)' space(:,3)'])
            pause(0.2)
            for m=1:9
                %delete(h(m));
            end
        end
        else
            theta=vertices(i,:);
            T=fkine_UR10(theta,0);
            [SetPointS]=Joints0(BasePoint,angle,T);
            h=draw_armA(SetPointS,'g');
            axis equal
            axis([space(:,1)' space(:,2)' space(:,3)'])
            pause(0.2)
            for m=1:9
                %delete(h(m));
            end
        end
            
    end
end