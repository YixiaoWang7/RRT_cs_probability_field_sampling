clc;
load map.mat;
p_start = [206, 198];%start point
p_goal = [416, 612];%goal point
k = 10000;%iterations
deltad = 50;%distance of the branches
P = 0.3;%the posibility of the random point being the goal point


[vertices, edges, path] = rrt_connect(map, p_start, p_goal, k, deltad, P);



%[vertices, edges, path] = rrt(map, p_start, p_goal, k, deltad, P);
%delta = 5;
%path_smooth = smooth(map, path, vertices, delta);



%»­ÈýÎ¬Ãæ
% size=200;
% origin=[200,0,100];
% 
% x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0]-0.5)*size+origin(1);
% y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1]-0.5)*size+origin(2);
% z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1]-0.5)*size+origin(3);
% for i=1:6
%     h=patch(x(:,i),y(:,i),z(:,i),'g');
%     set(h,'edgecolor','k','facealpha',0.5)
% end
% 
% axis equal
% axis([0 350 -150 150 0 250])
% grid on
% view(-33,18)