function [P_derivative_B_spline] = deboor_derivative(P,p,u,r,U,dimension)%r??????,
n=size(P,2)-1;
% U=vector(n,p);
for j=1:n+p
    if u>=U(j)&u<U(j+1)
        i=j; 
    end
end
if u==U(end)
    i=n+1;
end
d=zeros(dimension,n+1);
d(:,i-p:i)=P(:,i-p:i);
for l=1:r
    for j=i-p:i-l
        a=(p-l+1)/(U(j+p+1)-U(j+l));
        if U(j+p+1)-U(j+l)==0
            a=0;
        end
        d(:,j)=a*d(:,j+1)-a*d(:,j);
    end
end
for l=r+1:p
    for j=i-p:i-l
        a=(u-U(j+l))/(U(j+p+1)-U(j+l));
        if U(j+p+1)-U(j+l)==0
            a=0;
        end
        d(:,j)=a*d(:,j+1)+(1-a)*d(:,j);
    end
end
P_derivative_B_spline=d(:,i-p);

%  P_derivative_B_spline=deboor (d,p-r,u,U);