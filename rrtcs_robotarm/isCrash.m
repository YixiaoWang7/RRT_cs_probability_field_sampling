function [Crash]=isCrash(ob,theta,BasePoint,angle)
%whether arms crashes obstacles
Crash=0;
l=20;
T=fkine_UR10(theta,0);
[SetPointS]=Joints0(BasePoint,angle,T);
for i=1:9
    p1=SetPointS{i};
    p2=SetPointS{i+1};
    lp=(sum(p1.*p2))^0.5;
    n=lp/l;
    for j=0:n-1
        p=p1+(p2-p1)*j/n;
        for m=1:length(ob)
            if p(1)<ob{m}(2,1)&p(1)>ob{m}(1,1)
                if p(2)<ob{m}(2,2)&p(2)>ob{m}(1,2)
                    if p(3)<ob{m}(2,3)&p(3)>ob{m}(1,3)
                        Crash=1;
                        return
                    end
                end
            end
        end
    end
end
p=SetPointS{10};
for m=1:length(ob)
    if p(1)<ob{m}(2,1)&p(1)>ob{m}(1,1)
        if p(2)<ob{m}(2,2)&p(2)>ob{m}(1,2)
            if p(3)<ob{m}(2,3)&p(3)>ob{m}(1,3)
                Crash=1;
                return
            end
        end
    end
end


