% there are obstacles in this part.

%%world and show the world
%the whole space
close all
clear all
space=[-10 90;-5 25;-5 25];
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
axis([space(1,:) space(2,:) space(3,:)])
grid on
hold on
%obstacles
%first obstacle is ladder-liked
obstacles=cell(1,1);
ob=[];%form:3xn
for j=1:9
    for i=(1+floor(j/2)):5
        temp=[(20+i)*ones(1,10);5:14;(0+j)*ones(1,10)];
        ob=[ob temp];
    end
end
obstacles{1}=ob;
%second obstacle is blakeboard-liked
ob=[];
for i=1:8
    temp=[40*ones(1,16);3:18;(6+i)*ones(1,16)];
    ob=[ob temp];
end
temp=[40*ones(1,15);2*ones(1,15);0:14];
ob=[ob temp];
temp=[40*ones(1,15);19*ones(1,15);0:14];
ob=[ob temp];
obstacles{2}=ob;

ob=[];
for i=1:10
    if i<4 | i>7
        temp=[30*ones(1,10);[1:10]+3;(-3+i)*ones(1,10)];
        ob=[ob temp];
    else
        temp = [30*ones(1,10-2*(i-3)+1);[1:8-i i+2:10]+3;(-3+i)*ones(1,10-2*(i-3)+1)];
        ob=[ob temp];
    end
end
obstacles{3}=ob;

%show the world;the blue blocks are obstacles
Color=cell(1,4);
Color{1}='r';
Color{2}='g';
Color{3}='b';
Color{4}=[0.5,0.5,0.5];
for m=1:3
    for i=1:size(obstacles{m},2)
        center=obstacles{m}(:,i);
        drawcube(center,1,Color{4},0.2)
    end
end
%% origin and destination
type={1,2,3};
position={[1,5,1]',[1,10,1]',[1,15,1]'};
origin=struct('type',type,'position',position);
destination=struct('type',type,'position',{[79,5,1]',[79,10,1]',[79,15,1]'});
%show the goods and destination

origintype=1;
%origintype=rand()/2+1;%if you want different origin,change it.
center=origin(origintype).position;
drawcube(center,1,Color{1},0.5)%red

destinationtype=2;
%destinationtype=rand()/2+1;%if you want different origin,change it.
center=destination(destinationtype).position;
drawcube(center,1,Color{2},0.5)%green

%% RRT
k=10000;%iterations
deltad=3;%the length of each branch of tree
d=10;%the radius of ultra-dimensional ball

p_start=(origin(origintype).position)';
p_start_type=origin(origintype).type;p_goal=(destination(destinationtype).position)';
color=Color{3};%the path is blue
tic
[vertices, edges, path] = rrt_csd(space,obstacles, p_start, p_goal, k, deltad,d,color);%include rrtDraw
toc
pathpoint=(vertices(path,:))';
pathlength=0;%
for i=1:size(pathpoint,2)-1
    pathlength=pathlength+norm(pathpoint(:,i)-pathpoint(:,i+1));
end
pathlength
%% B-spline and velocity planning
if 1
dimension=3;
N=2000;
pp=4;
[C,Cu,Cuu,Cuuu,ROC,P,U,uu]=gen_B_spline(pp,N,pathpoint,dimension);
color=Color{3};
plot3(C(1,:),C(2,:),C(3,:),color,'LineWidth',2)
end

  


