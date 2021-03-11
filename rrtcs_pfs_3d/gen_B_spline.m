function [C,Cu,Cuu,Cuuu,ROC,P,U,u]=gen_B_spline(p,N,Q,dimension)
%p=4;N=2000;
nd=10;
if 0
    n=floor(size(Q,2)/nd);
    index=1;
    for i=1:n-1
        index=[index 1+nd*i];
    end
    if 1+nd*n<size(Q,2)
        index=[index 1+nd*n];
    else index=[index size(Q,2)];
    end
    Q=Q(:,index);
    % plot(Q(1,:),Q(2,:),'r*')
    % hold on
end
% p1=Q(:,1)*2-Q(:,2);
% Q=[p1 Q];
% size(Q,2);
n_path_point=size(Q,2)-1;
u=linspace(0,1,N);
[P,n,U]=generate_control_point(Q,n_path_point,p);
C=zeros(dimension,N);
Cu=zeros(dimension,N);
Cuu=zeros(dimension,N);
Cuuu=zeros(dimension,N);
ROC=zeros(1,N);
for i=1:N
    C(:,i)=deboor (P,U,p,u(i),dimension);
    Cu(:,i)=deboor_derivative(P,p,u(i),1,U,dimension);
    Cuu(:,i)=deboor_derivative(P,p,u(i),2,U,dimension);
    Cuuu(:,i)=deboor_derivative(P,p,u(i),3,U,dimension);
    ROC(i)=sum(Cu(:,i).*Cu(:,i))/sqrt(sum(Cuu(:,i).*Cuu(:,i))-(sum(Cuu(:,i).*Cu(:,i)))^2/sum(Cu(:,i).*Cu(:,i)));
end
% index=find(u>=U(p+1)&u<U(p+2));
% u(index)=[];
% C(:,index)=[];
% Cu(:,index)=[];
% Cuu(:,index)=[];
% Cuuu(:,index)=[];
% ROC(index)=[];
% plot(Q(1,:),Q(2,:),'*')
% hold on