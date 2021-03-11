function [FinalV,FinalS]=BatA(In,Bd,InP,para)
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
LocR=40;%Local search range
Popv=zeros(InP,Dim);%velocity


PopV=zeros(InP,1);%value
Pop=zeros(InP,Dim);
for i=1:InP
    while 1
        Pop(i,:)=Bd(1,:)+(Bd(2,:)-Bd(1,:)).*rand(1,Dim);
        PopV(i)=BaFun([In(1,:) Pop(i,:) In(2,:)],para);
        if PopV(i)<100000
            break
        end
    end

end


[BestV,BestIn]=min(PopV);
tempV=BestV;
BestS=Pop(BestIn(1),:);
BestSet=zeros(MaxI,1);
cri=0;
for j=1:MaxI
    for i=1:InP
        f(i)=fmin+(fmax-fmin)*rand();
        Popv(i,:)=Popv(i,:)+(Pop(i,:)-BestS)*f(i);
        NewS=Pop(i,:)+Popv(i,:);
        if rand()>r(i)
            temprand=rand(1,Dim);
            Index=randperm(Dim/2);
            Index=Index(1);
            temprand=temprand(Index*2-1:Index*2);
            NewS=[NewS;BestS+LocR*(rand(1,Dim)-0.5)];
        else
            temp=randperm(InP);
            tempp=Pop(temp(1),:)+F*(Pop(temp(2),:)-Pop(temp(3),:));
            NewS=[NewS;tempp];
            
        end
        NewS=[NewS;Bd(1,:)+(Bd(2,:)-Bd(1,:)).*rand(1,Dim)];
        for n=1:size(NewS,2)/2
            for m=1:2
                Index=find(NewS(:,2*n-2+m)<Bd(1,2*n-2+m));
                NewS(Index,2*n-2+m)=Bd(1,2*n-2+m)*ones(length(Index),1);            
                Index=find(NewS(:,2*n-2+m)>Bd(2,2*n-2+m));
                NewS(Index,2*n-2+m)=Bd(2,2*n-2+m)*ones(length(Index),1);
            end
        end
        NewV=[];
        for m=1:size(NewS,1)
            NewV=[NewV BaFun([In(1,:) NewS(m,:) In(2,:)],para)];
        end
        [NewV,NewIn]=min(NewV);
        NewS=NewS(NewIn(1),:);
        if (NewV<PopV(i))&(rand()<A(i))
            Pop(i,:)=NewS;
            PopV(i)=NewV;
            A(i)=Af*A(i);
            r(i)=r0*(1-exp(-Rf*iter));
            if NewV<=BestV
                BestV=NewV;
                BestS=NewS;
                PathPoint=[];
                for m=1:Dim/2
                    tempp=flip(BestS(2*m-1:2*m)');
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



end