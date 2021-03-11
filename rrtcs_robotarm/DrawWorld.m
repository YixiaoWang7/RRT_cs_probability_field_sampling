function DrawWorld
theta=[0 0 0 0 0 0];
theta_in=[-90 90 0 -90 0 0];
theta=theta+theta_in;
T=fkine_UR10(theta,0);
BasePoint=[1500,0,1000]';
angle=0*pi/180;
[SetPointS]=Joints0(BasePoint,angle,T);
draw_arm(SetPointS)
hold on


%creat the world
xl=[-1000,3000];
yl=[-1000,2500];
zl=[-1000,3000];
space=[xl' yl' zl'];
ob=cell(1,2);%obstacles

size=100;
origin=[1480,600,1800];
x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size+origin(1);
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size+origin(2);
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{1}=[origin(1)-size/2;origin(1)+size/2];
for i=2:3
    ob{1}=[ob{1} [origin(i)-size/2;origin(i)+size/2]];
end

size=100;
origin=[1500,1000,1070];
x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size+origin(1);
y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size+origin(2);
z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{2}=[origin(1)-size/2;origin(1)+size/2];
for i=2:3
    ob{2}=[ob{2} [origin(i)-size/2;origin(i)+size/2]];
end

size=40;
origin=[1500,1200,1000];
cuboidxyz=[20,20,1];
pset=[0 0 0 0 1 1 1 1;0 0 1 1 0 0 1 1;0 1 1 0 0 1 1 0];
for i=1:3
    pset(i,:)=pset(i,:)*cuboidxyz(i);
end
center=cuboidxyz/2;
inface=[1 2 3 4;1 2 6 5;6 5 8 7;2 6 7 3;7 8 4 3;1 5 8 4];
x=([pset(1,inface(1,:))' pset(1,inface(2,:))' pset(1,inface(3,:))' pset(1,inface(4,:))' pset(1,inface(5,:))' pset(1,inface(6,:))']-center(1))*size+origin(1);
y=([pset(2,inface(1,:))' pset(2,inface(2,:))' pset(2,inface(3,:))' pset(2,inface(4,:))' pset(2,inface(5,:))' pset(2,inface(6,:))']-center(2))*size+origin(2);
z=([pset(3,inface(1,:))' pset(3,inface(2,:))' pset(3,inface(3,:))' pset(3,inface(4,:))' pset(3,inface(5,:))' pset(3,inface(6,:))']-center(3))*size+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{3}=[origin(1)-cuboidxyz(1)*size/2;origin(1)+cuboidxyz(1)*size/2];
for i=2:3
    ob{3}=[ob{3} [origin(i)-cuboidxyz(i)*size/2;origin(i)+cuboidxyz(i)*size/2]];
end


size=20;
origin=[1500,1200,490];
cuboidxyz=[1,1,49];
pset=[0 0 0 0 1 1 1 1;0 0 1 1 0 0 1 1;0 1 1 0 0 1 1 0];
for i=1:3
    pset(i,:)=pset(i,:)*cuboidxyz(i);
end
center=cuboidxyz/2;
inface=[1 2 3 4;1 2 6 5;6 5 8 7;2 6 7 3;7 8 4 3;1 5 8 4];
x=([pset(1,inface(1,:))' pset(1,inface(2,:))' pset(1,inface(3,:))' pset(1,inface(4,:))' pset(1,inface(5,:))' pset(1,inface(6,:))']-center(1))*size+origin(1);
y=([pset(2,inface(1,:))' pset(2,inface(2,:))' pset(2,inface(3,:))' pset(2,inface(4,:))' pset(2,inface(5,:))' pset(2,inface(6,:))']-center(2))*size+origin(2);
z=([pset(3,inface(1,:))' pset(3,inface(2,:))' pset(3,inface(3,:))' pset(3,inface(4,:))' pset(3,inface(5,:))' pset(3,inface(6,:))']-center(3))*size+origin(3);
for i=1:6
    h=patch(x(:,i),y(:,i),z(:,i),'g');
    set(h,'edgecolor','k','facealpha',0.5)
end
ob{4}=[origin(1)-cuboidxyz(1)*size/2;origin(1)+cuboidxyz(1)*size/2];
for i=2:3
    ob{4}=[ob{4} [origin(i)-cuboidxyz(i)*size/2;origin(i)+cuboidxyz(i)*size/2]];
end

axis equal
axis([xl yl zl])
xlabel('x')
ylabel('y')
zlabel('z')
grid on
hold on