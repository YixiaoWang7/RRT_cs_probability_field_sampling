function [isfree]=isFree2(ob,p_near,p_new,deltad)
    isfree=1;
    v = double(p_new - p_near);
    if v==0
        return
    end
    distance = norm(v);
    intermediatePointCount=floor(distance/deltad)*20;
    if intermediatePointCount<20
        intermediatePointCount=20;  
    end
    u = v / distance;
    deltad = distance / intermediatePointCount;
    currentCoordinate = double(p_near);
    if isempty(ob{1})
        isfree=1;
        return
    else
        for ii = 0 : intermediatePointCount
            currentCoordinate = currentCoordinate + (deltad * u);
            cC=round(currentCoordinate);
            for i=1:length(p_near)
                if cC(i)<1
                    cC(i)=1;
                elseif cC(i)>size(ob{1},i)
                    cC(i)=size(ob{1},i);
                end
            end
            for i=1:length(ob)
               
                tempob=ob{i};
%                 t1=find(tempob(1,:)==cC(1));
%                 if isempty(t1)
%                     continue
%                 else
%                     t2=find(tempob(2,t1)==cC(2));
%                     if isempty(t2)
%                         continue
%                     else
%                         t3=find(tempob(3,t1(t2))==cC(3));
%                         if isempty(t3)
%                             continue
%                         else
%                             isfree=0;
%                             return
%                         end
%                     end     
%                 end
                if length(p_near)==3
                    if tempob(cC(1),cC(2),cC(3))==0%black
                        isfree=0;
                        return
                    end
                elseif length(p_near)==2
                    if tempob(cC(1),cC(2))==0%black
                        isfree=0;
                        return
                    end
                end
            end
        end
    end
    isfree = 1;
end