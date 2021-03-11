function [P,n,U] = generate_control_point (Q,m,p)
ucontrol=linspace(0,1,m+1);
Q=Q';%??????????
dif=floor(size(Q,1)/2);
n=m-dif;%????????????????????????
c=(m+1)/(n-p+1);
U=zeros(1,n+p+2);
U(n+2:n+p+2)=ones(1,p+1);
for j=1:n-p
    i=floor(j*c);
    afa=j*c-i;
    U(p+j+1)=(1-afa)*ucontrol(i)+afa*ucontrol(i+1);
end
for i=1:m-1
    N=Nbasis(U,ucontrol(i+1),n,p);
    R(i,:)=Q(i+1,:)-Q(1,:)*N(1)-Q(end,:)*N(n+1);
end
for i=1:m-1
    N=Nbasis(U,ucontrol(i+1),n,p);
    D(i,:)=N(2:n);
end
% P=D^-1*Q';
% P=(D'*D)^-1*D'*Q';
P=(D'*D)^-1*(D'*R);
P=[Q(1,:);P;Q(end,:)];
P=P';
% P=P';
% P=[Q(:,1),P,Q(:,n+1)];