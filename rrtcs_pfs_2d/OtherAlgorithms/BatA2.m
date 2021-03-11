function [FinalV,FinalS]=BatA2(In,Bd,InP,para,l)
%Bd: 2 by dimension, boundary of the solution
%InP:Initial Population

iter=1;
MaxI=500;%max iterations
Dim=size(Bd,2);
F=0.3;
A=ones(InP,1);%loudness
r=zeros(InP,1);%pulse rate
r0=1;
Af=0.9;
Rf=0.3;
f=zeros(InP,Dim);
fmin=0;
fmax=0.5;
LocR=pi/3;%Local search range
Popv=zeros(InP,Dim);%velocity


PopV=zeros(InP,1);%value
Pop=zeros(InP,Dim);
plist=zeros(InP,Dim*2+4);
S=[9 9;235 235];
for i=1:InP
    [Pop(i,:),plist(i,:)]=GenPop(In,Dim,S,l);
    PopV(i)=BaFun2(plist(i,:),para,l);
end

[BestV,BestIn]=min(PopV);
tempV=BestV;
BestS=Pop(BestIn(1),:);

plist(BestIn(1),:)
PathPoint=[];
[path,~]=GenPath(In,BestS,S,l);
for m=1:Dim
    tempp=flip(path(2*m-1:2*m)');
    PathPoint=[PathPoint tempp];
end
PathPoint=[flip(In(1,:)') PathPoint flip(In(2,:)')];
plot(PathPoint(1,:),PathPoint(2,:),'g*-')
BestS/pi*180
pause




BestSet=zeros(MaxI,1);
cri=0;
for j=1:MaxI
    for i=1:InP
        f(i)=fmin+(fmax-fmin)*rand();
        Popv(i,:)=Popv(i,:)+(Pop(i,:)-BestS)*f(i);
        NewS=Pop(i,:)+Popv(i,:);
        if rand()>r(i)
            loc=BestS+LocR*(rand(1,Dim)-0.5);
            NewS=[NewS;BestS+LocR*(rand(1,Dim)-0.5)];
        else
            temp=randperm(InP);
            tempp=Pop(temp(1),:)+F*(Pop(temp(2),:)-Pop(temp(3),:));
            NewS=[NewS;tempp];
        end
        [tempp,~]=GenPop(In,Dim,S,l);
        NewS=[NewS;tempp];
        for n=1:size(NewS,2)/2
            for m=1:2
                Index=find(NewS(:,2*n-2+m)<Bd(1,2*n-2+m));
                NewS(Index,2*n-2+m)=Bd(1,2*n-2+m)*ones(length(Index),1);            
                Index=find(NewS(:,2*n-2+m)>Bd(2,2*n-2+m));
                NewS(Index,2*n-2+m)=Bd(2,2*n-2+m)*ones(length(Index),1);
            end
        end
        NewV=[];
        checklist=[];
        for m=1:size(NewS,1)
            [path,check]=GenPath(In,NewS(m,:),S,l);
            if check==0
                checklist=[checklist m];
                NewV=[NewV 100000];
            else
                NewV=[NewV BaFun2(path,para,l)];
            end
        end
        if length(checklist)==size(NewS,1)
            continue
        end
        NewV(checklist)=[];
        NewS(checklist,:)=[];
        [NewV,NewIn]=min(NewV);
        NewS=NewS(NewIn(1),:);
        if (NewV<PopV(i))&(rand()<A(i))
            Pop(i,:)=NewS;
            PopV(i)=NewV;
            A(i)=Af*A(i);
            r(i)=r0*(1-exp(-Rf*iter));
            if NewV<BestV
                BestV=NewV;
                BestS=NewS;
                PathPoint=[];
                [path]=GenPath(In,BestS,S,l);
                BestV
                for m=1:Dim
                    tempp=flip(path(2*m-1:2*m)');
                    PathPoint=[PathPoint tempp];
                end
                PathPoint=[flip(In(1,:)') PathPoint flip(In(2,:)')];
                plot(PathPoint(1,:),PathPoint(2,:),'g*-')
            end
        end
    end
    iter=iter+1;
    if abs(BestV-tempV)/tempV<1e-4
        cri=cri+1;
    else
        cri=0;
    end
    if cri>100
        break
    end
    tempV=BestV;
end

FinalS=BestS;
FinalV=BestV;
    function [path,flag]=GenPath(I,theta,S,l)
        path=I(1,:);
        p1=path;
        flag=1;
        for ii=1:length(theta)
            p2=p1+l*[cos(theta(ii)) sin(theta(ii))];
            if p2(1)<S(1,1)
                flag=0;
                return
                p2(1)=S(1,1);
            end
            if p2(1)>S(2,1)
                flag=0;
                return
                p2(1)=S(2,1);
            end
            if p2(2)<S(1,2)
                flag=0;
                return
                p2(2)=S(1,2);
            end
            if p2(2)>S(2,2)
                flag=0;
                return
                p2(2)=S(2,2);
            end
            path=[path p2];
            p1=p2;
        end
        path=[path I(2,:)];
        
    end
    function [thetalist,plist]=GenPop(I,D,S,l)
        plist=I(1,:);
        p1=plist;
        thetalist=[];
        for ii=1:D
            while 1
                ttheta=2*pi*rand();
                tp=p1+l*[cos(ttheta) sin(ttheta)];
                if tp(1)>=S(1,1)&&tp(1)<=S(2,1)&&tp(2)>=S(1,2)&&tp(2)<=S(2,2)
                    p2=tp;
                    plist=[plist p2];
                    thetalist=[thetalist ttheta];
                    p1=p2;
                    break
                end
            end
        end
        plist=[plist I(2,:)];
        
    end


end