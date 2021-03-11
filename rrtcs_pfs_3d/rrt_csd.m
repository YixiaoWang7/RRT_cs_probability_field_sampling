function [vertices, edges, path] = rrt_csd(space,ob, p_start, p_goal, k, deltad,d,COLOR)
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
        [vertices,edges,verticeso,edgeso,connect,cost,costo]=ExtendB(space,ob,deltad,vertices,edges,verticeso,edgeso,cost,costo,d);
    else
        [verticeso,edgeso,vertices,edges,connect,costo,cost]=ExtendB(space,ob,deltad,verticeso,edgeso,vertices,edges,costo,cost,d);
    end
    if connect
%         rrtDrawEdges(vertices, edges,'g')
%         rrtDrawEdges(verticeso, edgeso,'b')
%         pause
        path=pathsolution1(edges);
%         rrtDraw( space,p_start, p_goal, vertices, edges, path)
%         figure
        patho=pathsolution1(edgeso);
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
        [path]=PathOptimal(path,ob,deltad,vertices);
        edgeso=flip(flip(edgeso,2));
        edges=[edges;[edgeso(1,2) edges(end,1)];edgeso];
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
%         
         rrtDraw(vertices, path,'r')
         rrtDrawEdges(vertices, edges,'g')
%         pause
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

function [vertices,edges,verticeso,edgeso,connect,cost,costo]=ExtendB(space,ob,deltad,vertices,edges,verticeso,edgeso,cost,costo,d)
    connect=0;
    while 1
        p_rand = (space(:,1)+(space(:,2)-space(:,1)).*rand(3,1))';
        if rand()<probabilityfield(ob,p_rand,0.5,0.8,1,3)
            break 
        end
    end
    [p_near, pNearIndex] = FindPNear(p_rand, vertices);
    [p_new, ~] = FindPNew(p_near, p_rand, deltad);
    
    
    
    
    if isFree(ob,p_near,p_new,deltad)==1
        vertices = [vertices; p_new];
        [pNewIndex, ~] = size(vertices);
        [edges,cost]=star(pNewIndex,pNearIndex,p_new,p_near,edges,cost,vertices,deltad,d,ob);
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
                    [edgeso,costo]=star(pNewIndex,pNearIndex,p_newo,p_nearo,edgeso,costo,verticeso,deltad,d,ob);
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

function [possibility]=probabilityfield(ob,point,p0,p1,pmax,d0)
    %point 1x3; ob 3x1
    if pmax>1
        error('pmax should be less than 1.')
    end
    if (p0>pmax)|(p1>pmax)
        error('p0,p1 should be less than pmax.')
    end
    
    dset=[];
    if isempty(ob{1})
        possibility=p1;
        return
    end
    for i=1:length(ob)
        for j=1:size(ob{i},2)
            tempd=norm(point-(ob{i}(:,j))');
            dset=[dset tempd];
        end
    end
    dmin=min(dset)-0.5;
    a=1/pmax;
    b=p0/(1-a*p0);
    beta=(1/p1-a)^0.5+1;
    if dmin<=0
        possibility=1/(a+1/b);
    elseif dmin>beta*d0
        possibility=1/((beta-1)^2/b+a);
    else
        possibility=1/((dmin-d0)^2/(d0^2*b)+a);
    end
end


function [edges,cost]=star(pNewIndex,pNearIndex,p_new,p_near,edges,cost,vertices,deltad,d,ob)
    edges = [edges; [int32(pNewIndex), int32(pNearIndex)]];
    cost(pNewIndex)=cost(pNearIndex)+norm(p_new-p_near);
    [p_n, pNIndex] = FindPNearSet(p_new, vertices,d);
    %to pn then to pnew
    for i=1:length(pNIndex)
        if pNIndex(i)~=pNearIndex
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
        if pNIndex(i)~=pNearIndex
            if isFree(ob,p_n(i,:),p_new,deltad)==1
                if cost(pNIndex(i))>cost(pNewIndex)+norm(p_n(i,:)-p_new)
                    in=find(edges(:,1)==pNIndex(i));
                    tempmark=0;
                    tempin=[];
                    for j=1:length(in)
                        if isFree(ob,vertices(edges(in(j),2),:),p_new,deltad)==1
                            tempmark=1;  
                            cost(pNIndex(i))=cost(pNewIndex)+norm(p_n(i,:)-p_new);
                            tempin=[tempin j];
                        end
                    end
                    if tempmark==1
                        edges(in(tempin),:)=[];
                        edges=[edges; [int32(pNIndex(i)), int32(pNewIndex)]];
                    end
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

function [isfree]=isFree(ob,p_near,p_new,deltad)
    v = double(p_new - p_near);
    distance = norm(v);
    intermediatePointCount=floor(distance/deltad)*20;
    if intermediatePointCount<20
        intermediatePointCount=20;  
    end
    u = v / distance;
    deltad = distance / intermediatePointCount;
    currentCoordinate = double(p_near);
    if isempty(ob{1})
        isFree=1;
        return
    else
        for ii = 0 : intermediatePointCount
            currentCoordinate = currentCoordinate + (deltad * u);
            cC=round(currentCoordinate);
            for i=1:length(ob)
                tempob=ob{i};
                t1=find(tempob(1,:)==cC(1));
                if isempty(t1)
                    continue
                else
                    t2=find(tempob(2,t1)==cC(2));
                    if isempty(t2)
                        continue
                    else
                        t3=find(tempob(3,t1(t2))==cC(3));
                        if isempty(t3)
                            continue
                        else
                            isfree=0;
                            return
                        end
                    end     
                end
            end
        end
    end
    isfree = 1;
end

function [path]=pathsolution1(edges)
    path(1)=edges(end,1);
    while 1
        index=find(edges(:,1)==path(end));
        path=[path edges(index(1),2)];
        if path(end)==1
            break
        end
    end
end

function [pathnew]=PathOptimal(path,ob,deltad,vertices)
    ci=1;%current index
    ti=ci;
    pathnew=path(ci);
    while 1
        ti=ti+1;%temp index
        if isFree(ob,vertices(path(ci),:),vertices(path(ti),:),deltad)
            if ti==length(path)
                pathnew=[pathnew path(end)];
                break
            else
                continue
            end
        else
            if ci~=ti-1
                pathnew=[pathnew path(ti-1)];
            end
            ci=ti-1;
        end
        if ti==length(path)
            pathnew=[pathnew path(end)];
            break
        end
    end
end

function rrtDraw(vertices, path,color)
    hold on;
    grid on
    [~, pathCount] = size(path);
    if size(vertices,2)~=3
        error('DrawCode should be changed.current code is for 3D of extended tree.')
    end
    for i = 1 : pathCount-1
        p=(vertices(path(i:i+1),:))';
        plot3(p(1,:),p(2,:),p(3,:),[color '--'],'LineWidth',2)
%         plot3(p(1,:),p(2,:),p(3,:),[color '*'])
    end
end

function rrtDrawEdges(vertices, edges,color)
    hold on;
    grid on
    [Count,~] = size(edges);
    if size(vertices,2)~=3
        error('DrawCode should be changed.current code is for 3D of extended tree.')
    end
    for i = 1 : Count
        p=(vertices(edges(i,:),:))';
        plot3(p(1,:),p(2,:),p(3,:),[color '-'])
        plot3(p(1,:),p(2,:),p(3,:),[color '*'])
    end
end


