function [vertices, edges, path] = rrt_star(space,ob, p_start, p_goal, k, deltad,d,COLOR)
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
cost(1)=double(0);

tic
for ii = 1 : k % For k samples repeat
    [vertices,edges,cost]=ExtendB(space,ob,deltad,vertices,edges,cost,d);
    if mod(ii,500)==0
        imshow('./maze2.jpg','InitialMagnification',200)
        rectangle('position',[p_start-5 10 10],'Curvature',1,'FaceColor',[0 0 1],'EdgeColor',[0 0 1])
        rectangle('position',[p_goal-5 10 10],'Curvature',1,'FaceColor',[1 0 0],'EdgeColor',[1 0 0])
        rrtDrawEdges(vertices, edges,'g')
        pause
    end
    if norm(vertices(end,:)-p_goal)<deltad
        vertices=[vertices;p_goal];
        edges=[edges;[size(vertices,1) size(vertices,1)-1]];
        path1=pathsolution(edges);
        [path]=PathOptimal(path1,ob,deltad,vertices);
%         (length(path1)-1)*deltad %path length for path1
%         ll=0;
%         for i=1:length(path)-1
%             ll=ll+norm(vertices(path(i),:)-vertices(path(i+1),:));
%         end
%         ll %path length for optimal path
%         %add new points to let the distance between two adjoining new
%         %pathpoints almost equal to deltad.
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
        toc
        ii
%         rrtDrawEdges(vertices, edges,'g')
%         rrtDrawPath(vertices, path,'r')
        return
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

function [vertices,edges,cost]=ExtendB(space,ob,deltad,vertices,edges,cost,d)
    p_rand = (space(:,1)+(space(:,2)-space(:,1)).*rand(size(space,1),1))';
    [p_near, pNearIndex] = FindPNear(p_rand, vertices);
    [p_new, ~] = FindPNew(p_near, p_rand, deltad);
    if isFree(ob,p_near,p_new,deltad)==1
        vertices = [vertices; p_new];
        [pNewIndex, ~] = size(vertices);
        [edges,cost]=star(pNewIndex,pNearIndex,p_new,p_near,edges,cost,vertices,deltad,d,ob);
    end
end

function [edges,cost]=star(pNewIndex,pNearIndex,p_new,p_near,edges,cost,vertices,deltad,d,ob)
    edges = [edges; [int32(pNewIndex), int32(pNearIndex)]];
    cost(pNewIndex)=cost(pNearIndex)+norm(p_new-p_near);
    [p_n, pNIndex] = FindPNearSet(p_new, vertices,d);
    %to pn then to pnew
    for i=1:length(pNIndex)
        if pNIndex(i)~=pNearIndex&pNIndex(i)~=pNewIndex
            if isFree(ob,p_n(i,:),p_new,deltad)==1
                if cost(pNIndex(i))+norm(p_n(i,:)-p_new)<cost(pNewIndex)
                    cost(pNewIndex)=cost(pNIndex(i))+norm(p_n(i,:)-p_new);
                    edges(end,:)=[int32(pNewIndex), int32(pNIndex(i))];
                end
            end
        end
    end
    %to pnew then to pn
    for i=1:length(pNIndex)
        if pNIndex(i)~=pNearIndex&pNIndex(i)~=pNewIndex
            if isFree(ob,p_n(i,:),p_new,deltad)==1
                if cost(pNIndex(i))>cost(pNewIndex)+norm(p_n(i,:)-p_new)
                    in=find(edges(:,1)==pNIndex(i));%find its parent
                    cost(pNIndex(i))=cost(pNewIndex)+norm(p_n(i,:)-p_new);
                    edges(in,:)=[];
                    edges=[edges; [int32(pNIndex(i)), int32(pNewIndex)]];
%                     tempmark=0;
%                     tempin=[];
%                     for j=1:length(in)
%                         if isFree(ob,vertices(edges(in(j),2),:),p_new,deltad)==1
%                             tempmark=1;  
%                             cost(pNIndex(i))=cost(pNewIndex)+norm(p_n(i,:)-p_new);
%                             tempin=[tempin j];
%                         end
%                     end
%                     if tempmark==1
%                         edges(in(tempin),:)=[];
%                         edges=[edges; [int32(pNIndex(i)), int32(pNewIndex)]];
%                     end
                end
            end
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


function [p_n, pNIndex] = FindPNearSet(p_new, vertices,d)
    [rowCount, ~] = size(vertices);
    if rowCount < 1
        error('RRT: the number of vertices is not positive.');
    end
    EuclideanDistances = double.empty(0, 1);
    l=1;
    for i = 1 : rowCount
        EuclideanDistances(i, 1) = pdist2(double(p_new), double(vertices(i, :)), 'euclidean');
        if EuclideanDistances(i, 1)<d
            p_n(l,:)=vertices(i, :);
            pNIndex(l)=i;
            l=l+1;
        end
    end
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












