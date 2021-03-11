function [cm,plist]=AstarIn(s,t,map)
% s: start 1by2
% t: taget 1by2
tic
X=size(map,2);
Y=size(map,1);
closemap=double(map);
dis_p=1./zeros(size(map,1),size(map,2));
dis_p(s(1),s(2))=0;
openlist=[s s(1)+s(2)*Y];
current=[];
tree=[s(1)+s(2)*Y s(1)+s(2)*Y];
prenum=size(openlist,1);
dis_f_list=hcost(s,t);
closegrey=0.8;

while 1
endmark=0;
if prenum<size(openlist,1)
    for i=prenum+1:size(openlist,1)
        p=openlist(i,:);
        h=hcost(p(1:2),t);
        if h==0
            endmark=1;
            break
        end
        dis_f_list=[dis_f_list dis_p(p(1),p(2))+h];
    end
end
if endmark==1
    break
end
index=find(dis_f_list==min(dis_f_list));
current=openlist(index(1),1:2);
closemap(current(1),current(2))=closegrey;
p=current;
pn=p(1)+p(2)*Y;
openlist(index(1),:)=[];
dis_f_list(index(1))=[];
prenum=size(openlist,1);

% if ~isempty(tree)&mod(size(tree,1),500)==0
% imshow(closemap,'InitialMagnification',200)
% rectangle('position',[flip(s)-5 10 10],'Curvature',1,'FaceColor',[0 0 1],'EdgeColor',[0 0 1])
% rectangle('position',[flip(t)-5 10 10],'Curvature',1,'FaceColor',[1 0 0],'EdgeColor',[1 0 0])
% tlist=findlist(tree,p(1)+p(2)*Y);
% t2=floor(tlist/Y);
% t1=tlist-t2*Y;
% line(t2,t1,'color',[1 0 0])
% pause(0.02)
% end


for m=-1:1
    for n=-1:1
        if ~(m==0&n==0)
            tp=p+[m,n];
            tpn=tp(1)+tp(2)*Y;
            if tp(1)>0&tp(1)<=size(map,1)&tp(2)>0&tp(2)<=size(map,2)
                if map(tp(1),tp(2))~=0
                    if closemap(tp(1),tp(2))~=closegrey
                        
                        if abs(m)+abs(n)==2
                            if dis_p(tp(1),tp(2))>dis_p(p(1),p(2))+14
                                index=find(openlist(:,3)==tpn);
                                if isempty(index)
                                    openlist=[openlist;tp(1),tp(2),tpn];
                                else
                                    dis_f_list(index)=dis_f_list(index)-dis_p(tp(1),tp(2))+dis_p(p(1),p(2))+14;
                                end
                                dis_p(tp(1),tp(2))=dis_p(p(1),p(2))+14;
                                index=find(tree(:,2)==tpn);
                                tree(index,:)=[];
                                tree=[tree;pn,tpn];
                            end
                        else
                            if dis_p(tp(1),tp(2))>dis_p(p(1),p(2))+10
                                index=find(openlist(:,3)==tpn);
                                if isempty(index)
                                    openlist=[openlist;tp(1),tp(2),tpn];
                                else
                                    dis_f_list(index)=dis_f_list(index)-dis_p(tp(1),tp(2))+dis_p(p(1),p(2))+10;
                                end
                                dis_p(tp(1),tp(2))=dis_p(p(1),p(2))+10;
                                index=find(tree(:,2)==tp(1)+tp(2)*Y);
                                tree(index,:)=[];
                                tree=[tree;p(1)+p(2)*Y,tp(1)+tp(2)*Y];
                            end
                        end
                    end
                end
            end
        end
    end
end
end
toc
if ~isempty(tree)
imshow(closemap,'InitialMagnification',200)
rectangle('position',[flip(s)-5 10 10],'Curvature',1,'FaceColor',[0 0 1],'EdgeColor',[0 0 1])
rectangle('position',[flip(t)-5 10 10],'Curvature',1,'FaceColor',[1 0 0],'EdgeColor',[1 0 0])
tlist=findlist(tree,p(1)+p(2)*Y);
t2=floor(tlist/Y);
t1=tlist-t2*Y;
line(t2,t1,'color',[1 0 0])
end




cm=dis_p(t(1),t(2));
child=t(2)*Y+t(1);
list=findlist(tree,child);
t2=floor(list/Y);
t1=list-t2*Y;
plist=[t1;t2];
    function [l]=findlist(tt,cc)
        l=cc;
        while 1
        index=find(tt(:,2)==cc);
        parent=tt(index(1),1);
        l=[parent l];
        cc=parent;
        if cc==tt(1,1)
            break
        end
        end
    end
    

    function [H]=hcost(p1,p2)
%         Ha=abs(p1(1)-p2(1))+abs(p1(2)-p2(2));
%         Di=min(abs(p1(1)-p2(1)),abs(p1(2)-p2(2)));
%         H=Di*14+(Ha-Di*2)*10;
%         H=round(10*norm(p1-p2));
        H=10*(abs(p1(1)-p2(1))+abs(p1(2)-p2(2)));
    end


end
