function [N] = Nbasis(U,u,n,p)
N=zeros(1,n+p+1);
flag=1;
i=1;
limit=length(U);
while(flag)
    if u>=U(i)&u<U(i+1)
        N(i)=1;
        flag=0;
    end
    i=i+1;
    if i==limit
        flag=0;
    end
end
if u==U(end)
    N(n+1)=1;
end
for j=1:p
    for i=1:n+p+1-j
        a=(u-U(i))/(U(i+j)-U(i));
        b=(U(i+j+1)-u)/(U(i+j+1)-U(i+1));
        if U(i+j)-U(i)==0
            a=0;
        end
        if U(i+j+1)-U(i+1)==0
            b=0;
        end
        N(i)=a*N(i)+b*N(i+1);
    end
end

        
        