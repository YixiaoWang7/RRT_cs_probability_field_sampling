function [vertices, edges, path] = rrt_connect(space,ob, p_start, p_goal, k, deltad,COLOR)
%space is the area range, like [xmin xmax;ymin ymax;zmin zmax;...];ob are
%obstacles,which is a cell containing obstacles in the octree(3d)
%form;p_start/goal is the start/destination point;delta is the length of branches;
%k is number of iteration;delatad is the length of each branch in the tree
if nargin < 5
    error('First 3 parameter is required: space,obstacles, start and goal coordinates.');
elseif nargin < 6
    k = 10000;
    deltad = 50;
    d=5;
elseif nargin < 7
    deltad = 50;
    d=5;
elseif nargin < 8
    d=5;
end
checkParameters(space,ob, p_start, p_goal, k, deltad);
SpaceD=size(space,2);%dimension of space, 2d or 3d
RRTD=size(p_goal,2);%dimension of the extended tree, configuration space or some other space

p_start = double(p_start); 
p_goal = double(p_goal); 
vertices = p_start;
edges = int32.empty(0, 2); 
p_rand = double.empty(0, RRTD);
p_near = double.empty(0, RRTD);
p_new = double.empty(0, RRTD);
po = p_goal;%the opposite direction of extended tree, ideas from RRT-connect
p_goalo = p_start; 
cost(1)=double(0);
costo(1)=double(0);
verticeso = p_goal; 
edgeso = int32.empty(0, 2);

flag=1;%if flag=0, sampling from vericeso
connect=0;
tic
for ii = 1 : k % For k samples repeat
    if flag==1
        [vertices,edges,verticeso,edgeso,connect]=ExtendB(space,ob,deltad,vertices,edges,p_goal,verticeso,edgeso);
    else
        [verticeso,edgeso,vertices,edges,connect]=ExtendB(space,ob,deltad,verticeso,edgeso,p_start,vertices,edges);
    end
%     if mod(ii,1000)==0
%     rrtDrawEdges(vertices, edges,'g');
%     rrtDrawEdges(verticeso, edgeso,'g');
%     pause
%     end
    if connect
        toc
%         pause
        path=pathsolution(edges);
%         rrtDraw( space,p_start, p_goal, vertices, edges, path)
%         figure
        patho=pathsolution(edgeso);
%         rrtDraw( space,p_start, p_goal, verticeso, edgeso, patho)
%         figure
        num=size(vertices,1);
        vertices=[vertices;verticeso];
        for m=1:size(edgeso,1)
            for n=1:size(edgeso,2)
                edgeso(m,n)=edgeso(m,n)+num;
            end
        end
        for m=1:length(patho)
            patho(m)=patho(m)+num;
        end
        path=[flip(path) patho];
        edgeso=flip(flip(edgeso,2));
        edges=[edges;[edgeso(1,2) edges(end,1)];edgeso];
        return
        rrtDrawPath(vertices, path,'r')
        [path]=PathOptimal(path,ob,deltad,vertices);

        i=1;
        while 1
            dtemp=norm(vertices(path(i),:)-vertices(path(i+1),:));
            n=floor(dtemp/deltad);
            if n>1
                verticesplus=zeros(n-1,RRTD);
                pathplus=size(vertices,1)+1:size(vertices,1)+n-1;
                for j=1:n-1
                    verticesplus(j,:)=vertices(path(i),:)-(vertices(path(i),:)-vertices(path(i+1),:))*j/n;
                end
                vertices=[vertices;verticesplus];
                path=[path(1:i) pathplus path(i+1:end)];
                i=i+n;
            else
                i=i+1;
            end
            if i>length(path)-1
                break
            end
        end
%         ii
%         rrtDrawEdges(vertices, edges,'g')
%         rrtDrawPath(vertices, path,'r')
        return
    end
    if length(vertices)<=length(verticeso)
        flag=1;
    else
        flag=0;
    end
    
end
    toc;
    error('RRT: solution not found :');
end



function checkParameters(space,ob, p_start, p_goal, k, deltad)
    [SpaceRow,SpaceCol]=size(space);
    SpaceDimension=SpaceRow;
    if SpaceCol~=2
        error('rows of space should be minimum and maximum of one dimension');
    end
    NumOb=length(ob);
    [x, y] = size(p_start);
    RRTDimension=y;
    if x ~= 1
        error('start position should be a row vector: [0, 0, 0,...]');
    end
    [x, y] = size(p_goal);
    if x ~= 1 
        error('start position should be a row vector: [0, 0, 0,...]');
    end
    if y~=RRTDimension
        error('RRTDimension of p_start and p_goal should be same');
    end
    if k < 1
        error('k - iteration count - parameter should be positive');
    end
    if deltad < 1
        error('deltad - step size - parameter should be positive');
    end
end

function [vertices,edges,verticeso,edgeso,connect]=ExtendB(space,ob,deltad,vertices,edges,p_goal,verticeso,edgeso)
    connect=0;
    p_rand = (space(:,1)+(space(:,2)-space(:,1)).*rand(size(space,1),1))';
    [p_near, pNearIndex] = FindPNear(p_rand, vertices);
    [p_new, ~] = FindPNew(p_near, p_rand, deltad);
    
    if isFree(ob,p_near,p_new,deltad)==1
        vertices = [vertices; p_new];
        [pNewIndex, ~] = size(vertices);
        edges=[edges;[pNewIndex pNearIndex]];
        [p_nearo, pNearIndex] = FindPNear(p_new, verticeso); 
        [p_newo,connect] = FindPNew(p_nearo, p_new, deltad);
        if connect==1
            if isFree(ob,p_nearo,p_newo,deltad)~=1
                connect=0;
                return
            end
        end
        if connect==0
            while(1)
                if isFree(ob,p_nearo,p_newo,deltad)==1
                    verticeso = [verticeso; p_newo];
                    [pNewIndex, ~] = size(verticeso);
                    edgeso=[edgeso;[pNewIndex pNearIndex]];
                    pNearIndex=pNewIndex;
                    p_nearo=p_newo;
                    [p_newo,connect] = FindPNew(p_nearo, p_new, deltad);
                    if connect==1
                        if isFree(ob,p_nearo,p_newo,deltad)~=1
                            connect=0;
                        end
                        return 
                    end
                else
                    break
                end
            end
        else
            return
        end
    end
end

function [p_near, pNearIndex] = FindPNear(p_rand, vertices)
    [rowCount, ~] = size(vertices);
    if rowCount < 1
        error('RRT: the number of vertices is not positive.');
    end
    EuclideanDistances = double.empty(0, 1);
    for i = 1 : rowCount
        EuclideanDistances(i, 1) = pdist2(double(p_rand), double(vertices(i, :)), 'euclidean');
    end
    minDistanceIndex = find(EuclideanDistances == min(EuclideanDistances));%find the nearest point to the new rand point
    p_near = vertices(minDistanceIndex(1), :);
    pNearIndex = minDistanceIndex(1);
end

function [p_new,connect] = FindPNew(p_near, p_rand, deltad)
    v = double(p_rand - p_near);
    dis=norm(v);
    u = v / dis;
    p_new = double(double(p_near) + deltad * u);
    if dis<deltad
        connect=1;
    else
        connect=0;
    end
end

