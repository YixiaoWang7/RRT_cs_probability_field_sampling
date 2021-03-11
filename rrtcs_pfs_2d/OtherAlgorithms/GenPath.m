    function [path]=GenPath(I,theta,S,l)
        path=I(1,:);
        p1=path;
        for ii=1:length(theta)
            p2=p1+l*[cos(theta(ii)) sin(theta(ii))];
            if p2(1)<S(1,1)
                p2(1)=S(1,1);
            end
            if p2(1)>S(2,1)
                p2(1)=S(2,1);
            end
            if p2(2)<S(1,2)
                p2(2)=S(1,2);
            end
            if p2(2)>S(2,2)
                p2(2)=S(2,2);
            end
            path=[path p2];
            p1=p2;
        end
        path=[path I(2,:)];
        
    end