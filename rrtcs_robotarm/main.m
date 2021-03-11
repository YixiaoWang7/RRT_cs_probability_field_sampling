close all
clear all
% create 3d world
theta=[0 0 0 0 0 0];
theta_in=[-90 90 0 -90 0 0];
theta=theta+theta_in;
T=fkine_UR10(theta,0);
BasePoint=[1500,0,1000]';
angle=0*pi/180;
[SetPointS]=Joints0(BasePoint,angle,T);
draw_arm(SetPointS);
hold on

%creat the world
xl=[-1000,3000];
yl=[-1000,2500];
zl=[-1000,3000];
space=[xl' yl' zl'];
ob=cell(1,2);%obstacles

side=40;
origin=[1500,1200,1000];
cuboidxyz=[20,20,1];
pset=[0 0 0 0 1 1 1 1;0 0 1 1 0 0 1 1;0 1 1 0 0 1 1 0];
for i=1:3
    pset(i,:)=pset(i,:)*cuboidxyz(i);
end
center=cuboidxyz/2;
inface=[1 2 3 4;1 2 6 5;6 5 8 7;2 6 7 3;7 8 4 3;1 5 8 4];
x=([pset(1,inface(1,:))' pset(1,inface(2,:))' pset(1,inface(3,:))' pset(1,inface(4,:))' pset(1,inface(5,:))' pset(1,inface(6,:))']-center(1))*side+origin(1);
y=([pset(2,inface(1,:))' pset(2,inface(2,:))' pset(2,inface(3,:))' pset(2,inface(4,:))' pset(2,inface(5,:))' pset(2,inface(6,:))']-center(2))*side+origin(2);
z=([pset(3,inface(1,:))' pset(3,inface(2,:))' pset(3,inface(3,:))' pset(3,inface(4,:))' pset(3,inface(5,:))' pset(3,inface(6,:))']-center(3))*side+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{1}=[origin(1)-cuboidxyz(1)*side/2;origin(1)+cuboidxyz(1)*side/2];
for i=2:3
    ob{1}=[ob{1} [origin(i)-cuboidxyz(i)*side/2;origin(i)+cuboidxyz(i)*side/2]];
end
side=20;
origin=[1500,1200,490];
cuboidxyz=[1,1,49];
pset=[0 0 0 0 1 1 1 1;0 0 1 1 0 0 1 1;0 1 1 0 0 1 1 0];
for i=1:3
    pset(i,:)=pset(i,:)*cuboidxyz(i);
end
center=cuboidxyz/2;
inface=[1 2 3 4;1 2 6 5;6 5 8 7;2 6 7 3;7 8 4 3;1 5 8 4];
x=([pset(1,inface(1,:))' pset(1,inface(2,:))' pset(1,inface(3,:))' pset(1,inface(4,:))' pset(1,inface(5,:))' pset(1,inface(6,:))']-center(1))*side+origin(1);
y=([pset(2,inface(1,:))' pset(2,inface(2,:))' pset(2,inface(3,:))' pset(2,inface(4,:))' pset(2,inface(5,:))' pset(2,inface(6,:))']-center(2))*side+origin(2);
z=([pset(3,inface(1,:))' pset(3,inface(2,:))' pset(3,inface(3,:))' pset(3,inface(4,:))' pset(3,inface(5,:))' pset(3,inface(6,:))']-center(3))*side+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{2}=[origin(1)-cuboidxyz(1)*side/2;origin(1)+cuboidxyz(1)*side/2];
for i=2:3
    ob{2}=[ob{2} [origin(i)-cuboidxyz(i)*side/2;origin(i)+cuboidxyz(i)*side/2]];
end
side=100;
origin=[1500,1000,1070];
x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*side+origin(1);
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*side+origin(2);
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*side+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{3}=[origin(1)-side/2;origin(1)+side/2];
for i=2:3
    ob{3}=[ob{3} [origin(i)-side/2;origin(i)+side/2]];
end
if 1
side=200;
origin=[1500,500,1800];
x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*side+origin(1);
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*side+origin(2);
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*side+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{4}=[origin(1)-side/2;origin(1)+side/2];
for i=2:3
    ob{4}=[ob{4} [origin(i)-side/2;origin(i)+side/2]];
end

end
if 1
    side=5;
    origin=[1500,700,1500];
    cuboidxyz=[100,100,30];
    pset=[0 0 0 0 1 1 1 1;0 0 1 1 0 0 1 1;0 1 1 0 0 1 1 0];
    for i=1:3
        pset(i,:)=pset(i,:)*cuboidxyz(i);
    end
    center=cuboidxyz/2;
    inface=[1 2 3 4;1 2 6 5;6 5 8 7;2 6 7 3;7 8 4 3;1 5 8 4];
    x=([pset(1,inface(1,:))' pset(1,inface(2,:))' pset(1,inface(3,:))' pset(1,inface(4,:))' pset(1,inface(5,:))' pset(1,inface(6,:))']-center(1))*side+origin(1);
    y=([pset(2,inface(1,:))' pset(2,inface(2,:))' pset(2,inface(3,:))' pset(2,inface(4,:))' pset(2,inface(5,:))' pset(2,inface(6,:))']-center(2))*side+origin(2);
    z=([pset(3,inface(1,:))' pset(3,inface(2,:))' pset(3,inface(3,:))' pset(3,inface(4,:))' pset(3,inface(5,:))' pset(3,inface(6,:))']-center(3))*side+origin(3);
    for i=1:6
        h=patch(x(:,i),y(:,i),z(:,i),'g');
        set(h,'edgecolor','k','facealpha',0.5)
    end
    ob{5}=[origin(1)-cuboidxyz(1)*side/2;origin(1)+cuboidxyz(1)*side/2];
    for i=2:3
        ob{5}=[ob{5} [origin(i)-cuboidxyz(i)*side/2;origin(i)+cuboidxyz(i)*side/2]];
    end
end

axis equal
axis([xl yl zl])
xlabel('x')
ylabel('y')
zlabel('z')
grid on
hold on
%% UR model
%get T06 from the position of the actuator

EndPoint=[1500;1000;1120];
ActP6=[0;0;92];
ActP0=EndPoint-BasePoint;
AngleStart=theta;
Rangle=[pi,0,0];
if 0
R=TransR(Rangle);
PT06=ActP0-R*ActP6;
T06=[R PT06;0 0 0 1];
[itheta]=ikine_UR10(T06);
AngleGoal=cell(1,1);
l=1;
for i=1:8
    if ~isempty(itheta{i})
        if ~isCrash(ob,itheta{i},BasePoint,angle)
            AngleGoal{l}=itheta{i};
            l=l+1;
%             T{i}
%             T=fkine_UR10(itheta{i},0);
%             [SetPointS]=Joints0(BasePoint,angle,T);
%             draw_arm(SetPointS)
        end
    end
end
end

n=1;
AngleGoal=cell(1,n);
for i=1:n
    Tangle=Rangle+[0,0,2*pi/n*(i-1)];
    AngleGoal{i}=BestAngleGoal(Tangle,ActP0,ActP6,AngleStart,ob,BasePoint,angle);
end
DisAngle=zeros(1,length(AngleGoal));
for i=1:length(AngleGoal)
    DisAngle(i)=norm(AngleGoal{i}-AngleStart);
end
DisMinIndex=find(DisAngle==min(DisAngle));%it can be showed
AngleGoalF=AngleGoal{DisMinIndex};

%% RRT probability field sampling
k = 30000;%iterations
deltad = 5;%distance of the branches
P = 0.3;%the posibility of the random point being the goal point
[vertices, edges, path] = rrt_csd(space,ob, AngleStart, AngleGoalF, k, deltad, P,BasePoint,angle,10);
%%
rrtDraw(space, vertices, path,BasePoint,angle);
%% Output
% PathV=vertices(path,:);
% cost=cost_angle(PathV);%cost.
% n=length(path);
% PathV=PathV-[zeros(n,1) 180*ones(n,1) zeros(n,4)];
% PathV=PathV/180*pi;
%xlswrite('PathV',PathV)

% for i=1:size(PathV,1)-1
%     norm(PathV(i,:)-PathV(i+1,:))/pi*180
% end



