% the main algorithm is modified rrt with probablity field sampling.
% A-star and Bat algorithms are realized, too. Uncomment the corresponding
% section and test by yourself.

% create 2d map

close all
clear all

% dense environment
map = imread('maze.png');
map=map(10:size(map,1)-10,10:size(map,2)-10);
map = im2bw(map);
s=[90,10];
t=[485,353];

% normal environment
% map = imread('maze2.jpg');
% map = im2bw(map);
% s=[15,205];
% t=[205,15];

% plot the environment
% imshow(map,'InitialMagnification',100);
% rectangle('position',[s-5 10 10],'Curvature',1,'FaceColor',[0 0 1],'EdgeColor',[0 0 1])
% rectangle('position',[t-5 10 10],'Curvature',1,'FaceColor',[1 0 0],'EdgeColor',[1 0 0])
% hold on

%% rrt
space=[1 size(map,1);1 size(map,2)];
p_start=flip(s);
p_goal=flip(t);
ob=cell(1,1);
ob{1}=map;
k=10000;
deltad=20;
d=40;
addpath('./lib_rrt')
%[vertices, edges, path] = rrt_star(space,ob, p_start, p_goal, k, deltad,d,'b');
%[vertices, edges, path] = rrt_connect(space,ob, p_start, p_goal, k, deltad,'b');
%[vertices, edges, path] = rrt(space,ob, p_start, p_goal, k, deltad,'b');
% [vertices, edges, path] = rrt_cs(space,ob, p_start, p_goal, k, deltad,d,'b');
[vertices, edges, path] = rrt_csfield(space,ob, p_start, p_goal, k, deltad,d,'b');
%[vertices, edges, path] = rrt_csd(space,ob, p_start, p_goal, k, deltad,d,'b');

% % path length
% l=0;
% for i=1:length(path)-1
%     l=l+norm(vertices(i,:)-vertices(i+1,:));
% end

%% plot
close all
imshow(map,'InitialMagnification',200); 
rectangle('position',[s-5 10 10],'Curvature',1,'FaceColor',[0 0 1],'EdgeColor',[0 0 1])
rectangle('position',[t-5 10 10],'Curvature',1,'FaceColor',[1 0 0],'EdgeColor',[1 0 0])
rrtDrawEdges(vertices, edges,'g')
rrtDrawPath(vertices, path,'r')
[path]=PathOptimal(path,ob,deltad,vertices);
rrtDrawPath(vertices, path,'b')

%% A star
% addpath('./OtherAlgorithms')
% %s`tart and target
% X=size(map,2);
% Y=size(map,1);
% start=flip(s);
% target=flip(t);
% [cm,list]=Astar(start,target,map);
% %[cm,list]=AstarIn(start,target,map);
% % InP=100;
% % np=10;
% % Bd=[];
% % for i=1:np
% %     temp=[9 9;size(map,1)-26 size(map,2)-7];
% %     Bd=[Bd temp];
% % end
% % In=[start;target];
% % [FinalV,FinalS]=BatA(In,Bd,InP,map);
% 
% % PathPoint=[];
% % for i=1:np
% %     tempp=flip(FinalS(2*i-1:2*i)');
% %     PathPoint=[PathPoint tempp];
% % end
% % PathPoint=[s' PathPoint t'];
% % plot(PathPoint(1,:),PathPoint(2,:),'r*-')

%% Bat with angle as variables
% InP=75;
% np=20;
% Bd=[];
% for i=1:np
%     temp=[0;2*pi];
%     Bd=[Bd temp];
% end
% In=[start;target];
% l=25;
% [FinalV,FinalS]=BatA2(In,Bd,InP,map,l);
% 
% 
% S=[9 9;235 235];
% PathPoint=[];
% [path]=GenPath(In,FinalS,S,l);
% for m=1:np+1
%     tempp=flip(path(2*m-1:2*m)');
%     PathPoint=[PathPoint tempp];
% end
% PathPoint=[PathPoint];
% plot(PathPoint(1,:),PathPoint(2,:),'ro-')




