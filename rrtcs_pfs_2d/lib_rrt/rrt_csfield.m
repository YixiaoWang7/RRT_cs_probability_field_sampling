function [vertices, edges, path] = rrt_csfield(space,ob, p_start, p_goal, k, deltad,d,COLOR)
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

rob=10;
Pob=0.1;
Ppass=0.7;

rtree=250;
Ptree=1;
Para=[rob Pob Ppass rtree Ptree];


OperateI=zeros(2*rtree+1,2*rtree+1);
for i=-rtree:rtree
    for j=-floor(sqrt(rtree^2-i^2)):floor(sqrt(rtree^2-i^2))
        OperateI(rtree+1+j,rtree+1+i)=Ptree-norm([i,j])^2/rtree^2*(Ptree-Ppass);
    end
end

OperateD=zeros(2*rob+1,2*rob+1);
for i=-rob:rob
    for j=-floor(sqrt(rob^2-i^2)):floor(sqrt(rob^2-i^2))
        OperateD(rob+1+j,rob+1+i)=Pob+norm([i,j])^2/rob^2*(Ppass-Pob);
    end
end

[row,col]=find(ob{1}==0);
thing=[row col];
field=Ppass*ones(space(1,2),space(2,2));
[field]=PF(field,thing,0,OperateD,Para);
field1=field;
field2=field;

tic
for ii = 1 : k % For k samples repeat
    disp(['iteration: ' num2str(ii)])
    if flag==1
        [vertices,edges,verticeso,edgeso,connect,cost,costo]=ExtendB(space,ob,deltad,vertices,edges,verticeso,edgeso,cost,costo,d,field1);
        field1=PF(field1,verticeso,1,OperateI,Para);%for verticeso
    else
        [verticeso,edgeso,vertices,edges,connect,costo,cost]=ExtendB(space,ob,deltad,verticeso,edgeso,vertices,edges,costo,cost,d,field2);
        field2=PF(field2,vertices,1,OperateI,Para);%for vertices
    end
    if mod(ii,500)==0
        imshow(field1,'InitialMagnification',100)
        title(['Magnification #1 when iteration = ' num2str(ii)])
        rrtDrawEdges(verticeso, edgeso,'g');
        pause
        imshow(field2,'InitialMagnification',100)
        title(['Magnification #2 when iteration = ' num2str(ii)])
        rrtDrawEdges(vertices, edges,'g');
        pause
    end
%     if mod(ii,1000)==0
%     map = imread('./maze.png');
%     map = im2bw(map);
%     map=map(10:size(map,1)-10,10:size(map,2)-10);
%     imshow(map,'InitialMagnification','fit'); 
%     rectangle('position',[flip(p_start)-5 10 10],'Curvature',1,'FaceColor',[0 0 1],'EdgeColor',[0 0 1])
%     rectangle('position',[flip(p_goal)-5 10 10],'Curvature',1,'FaceColor',[1 0 0],'EdgeColor',[1 0 0])
%     rrtDrawEdges(vertices, edges,'g');
%     rrtDrawEdges(verticeso, edgeso,'g');
%     pause
%     end
    if connect
%         rrtDrawEdges(vertices, edges,'g')
%         rrtDrawEdges(verticeso, edgeso,'b')
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
        toc
        return
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
        %[path]=PathOptimal(path,ob,deltad,vertices);
        
        ii
%         
%          rrtDrawPath(vertices, path,'r')
%          rrtDrawEdges(vertices, edges,'g')
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

function [vertices,edges,verticeso,edgeso,connect,cost,costo]=ExtendB(space,ob,deltad,vertices,edges,verticeso,edgeso,cost,costo,d,field)
    connect=0;
    for j=1:100
        p_rand = (space(:,1)+(space(:,2)-space(:,1)).*rand(size(space,1),1))';
        p_randt=round(p_rand);
        for i=1:size(p_randt,1)
            if p_randt(i)<1
                p_randt(i)=1;
            elseif p_randt(i)>space(i,2)
                p_randt(i)=space(i,2);
            end
        end
        if rand()>field(p_randt(1),p_randt(2))
            continue
        else 
            break
        end
    end
    [p_near, pNearIndex] = FindPNear(p_rand, vertices);
    [p_new, collide] = FindPNew(ob, p_near, p_rand, deltad);
    if collide
        return
    end
    vertices = [vertices; p_new];   
    [pNewIndex, ~] = size(vertices);
    [edges,cost]=star(pNewIndex,pNearIndex,p_new,p_near,edges,cost,vertices,deltad,d,ob);
    [p_nearo, pNearIndex] = FindPNear(p_new, verticeso); 
    [p_newo,collide] = FindPNew(ob, p_nearo, p_new, deltad);
    if collide
        return
    end
    while(1)
        verticeso = [verticeso; p_newo];
        [pNewIndex, ~] = size(verticeso);
        [edgeso,costo]=star(pNewIndex,pNearIndex,p_newo,p_nearo,edgeso,costo,verticeso,deltad,d,ob);
        if norm(p_new-p_newo)<deltad & isFree(ob,p_new,p_newo,deltad)
            connect = 1;
            return
        end
        pNearIndex=pNewIndex;
        p_nearo=p_newo;
        [p_newo,collide] = FindPNew(ob, p_nearo, p_new, deltad);
        if collide
            return
        end
    end

end


function [field]=PF(field,thing,operate,OperateM,Para)
%operate==1: add
%operate==0: decrease
if isempty(thing)
    return
end
f1=size(field,1);
f2=size(field,2);

rob=Para(1);
Pob=Para(2);
Ppass=Para(3);
rtree=Para(4);
Ptree=Para(5);

if operate==1
    for m=1:size(thing,1)
        for i=0:rtree
            block=0;
            for j=0:floor(sqrt(rtree^2-i^2))
                p=[round(thing(m,1))+i,round(thing(m,2))+j];
                if p(1)>0&p(1)<=f1&p(2)>0&p(2)<=f2
                    t=field(p(1),p(2));
                    if t>=Ppass
                       if t<OperateM(rtree+1+j,rtree+1+i)
                       field(p(1),p(2))=OperateM(rtree+1+j,rtree+1+i);
                       end
                    elseif t>Pob
                        field(p(1),p(2))=field(p(1),p(2))+t-Ppass;
                    else
                        block=1;
                        break
                    end
                end
            end
            if block==1&j==0
                break
            end
            for j=-1:-1:-floor(sqrt(rtree^2-i^2))
                p=[round(thing(m,1))+i,round(thing(m,2))+j];
                if p(1)>0&p(1)<=f1&p(2)>0&p(2)<=f2
                    t=field(p(1),p(2));
                    if t>=Ppass
                       if t<OperateM(rtree+1+j,rtree+1+i)
                       field(p(1),p(2))=OperateM(rtree+1+j,rtree+1+i);
                       end
                    elseif t>Pob
                        field(p(1),p(2))=field(p(1),p(2))+t-Ppass;
                    else
                        break
                    end
                end
            end
        end
        for i=-1:-1:-rtree
            block=0;
            for j=0:floor(sqrt(rtree^2-i^2))
                p=[round(thing(m,1))+i,round(thing(m,2))+j];
                if p(1)>0&p(1)<=f1&p(2)>0&p(2)<=f2
                    t=field(p(1),p(2));
                    if t>=Ppass
                       if t<OperateM(rtree+1+j,rtree+1+i)
                       field(p(1),p(2))=OperateM(rtree+1+j,rtree+1+i);
                       end
                    elseif t>Pob
                        field(p(1),p(2))=field(p(1),p(2))+t-Ppass;
                    else
                        block=1;
                        break
                    end
                end
            end
            if block==1&j==0
                break
            end
            for j=-1:-1:-floor(sqrt(rtree^2-i^2))
                p=[round(thing(m,1))+i,round(thing(m,2))+j];
                if p(1)>0&p(1)<=f1&p(2)>0&p(2)<=f2
                    t=field(p(1),p(2));
                    if t>=Ppass
                       if t<OperateM(rtree+1+j,rtree+1+i)
                       field(p(1),p(2))=OperateM(rtree+1+j,rtree+1+i);
                       end
                    elseif t>Pob
                        field(p(1),p(2))=field(p(1),p(2))+t-Ppass;
                    else
                        break
                    end
                end
            end
        end
    end
else
    for m=1:size(thing,1)
        for i=-rob:rob
            for j=-floor(sqrt(rob^2-i^2)):floor(sqrt(rob^2-i^2))
                p=[thing(m,1)+i,thing(m,2)+j];
                if p(1)>0&p(1)<=f1&p(2)>0&p(2)<=f2
                    if field(p(1),p(2))>OperateM(rob+1+j,rob+1+i)
                        field(p(1),p(2))=OperateM(rob+1+j,rob+1+i);
                    end
                end
            end
        end
    end
    
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
function [p_new,collide] = FindPNew(ob, p_near, p_rand, deltad)
    v = double(p_rand - p_near);
    dis=norm(v);
    u = v / dis;
    p_new = double(double(p_near) + deltad * u);
    collide = 0;
    if ~isFree(ob, p_near, p_new, deltad)
        collide = 1;
    end
end

