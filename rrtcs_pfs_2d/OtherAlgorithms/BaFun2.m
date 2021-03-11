function [y]=BaFun2(p,para,l)
n=length(p);
if mod(n,2)~=0
    error('Dimension should be even.')
end
y=0;
ob{1}=para;
for i=1:n/2-1
    p1=p(2*i-1:2*i);
    p2=p(2*i+1:2*i+2);
    if isFree2(ob,p1,p2,20)
        y=y+norm(p1-p2);
    else
        y=y+10000;
    end
end
if sum((p1-p2).*(p1-p2))>l^2
    y=y+100*(sum((p1-p2).*(p1-p2))-l^2);
end