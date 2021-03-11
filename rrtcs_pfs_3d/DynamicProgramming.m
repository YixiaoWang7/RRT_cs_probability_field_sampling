function [path,lengthofpath]=DynamicProgramming(edges,vertices)
    %edges should be nx2 matrix, first col means the parent point,the final
    %row means the destination;vertices should include length information.
    %path is a cell
    %let edges be
    %[7,5,4;7,2,5;7,6,3;5,2,6;5,1,4;5,3,1;6,4,2;2,1,8;3,1,3;4,1,9] the
    %third col means its length. it will output 1x2 cell path. calculation
    %about length d should be adjusted.
    path=cell(1,1);
    path{1}=edges(end,1);
    lengthofpath=0;
    while 1
        l=1;
        temppath=cell(1,1);
        templength=0;
        tempnextindex=[];
        mark=0;%means no update
        for i=1:length(path)
            lastindex=path{i}(end);
            if lastindex==1
                temppath{l}=path{i};
                templength(l)=lengthofpath(i);
                l=l+1;
                tempnextindex=[tempnextindex lastindex];
            else
                mark=1;
                in=find(edges(:,1)==lastindex);%the index of points which connect the current path
                for j=1:length(in)
                    nextindex=edges(in(j),2);
                    tempnextindex=[tempnextindex nextindex];
                    d=norm(vertices(lastindex,:)-vertices(nextindex,:));
%                     for q=1:size(vertices,1)
%                         if vertices(q,1)==lastindex
%                             if vertices(q,2)==nextindex
%                                 d=vertices(q,3);
%                             end
%                         end
%                     end
                    temppath{l}=[path{i} nextindex];
                    templength(l)=lengthofpath(i)+d;
                    l=l+1;
                end
            end
        end
        if mark==0
            break
        end
        RepeatMatrix=tabulate(tempnextindex);
        tempindex1=find(RepeatMatrix(:,2)==1);
        tempindex=find(RepeatMatrix(:,2)>1);
        if isempty(tempindex)
            path=temppath;
            lengthofpath=templength;
        else
            path={};
            lengthofpath=[];
            if ~isempty(tempindex1)
                for i=1:length(tempindex1)
                    tempi=find(tempnextindex==RepeatMatrix(tempindex1(i),1));
                    path{i}=temppath{tempi};
                    lengthofpath(i)=templength(tempi);
                end
            end
            RepeatValue=RepeatMatrix(tempindex,1);
            for i=1:length(RepeatValue)
                tempindex=find(tempnextindex==RepeatValue(i));
                tempindexmin=find(templength(tempindex)==min(templength(tempindex)));
                temppath{tempindex(tempindexmin)};
                path=[path temppath{tempindex(tempindexmin)}];
                lengthofpath=[lengthofpath templength(tempindex(tempindexmin))];
            end
        end
        mark=1;
        for i=1:length(path)
            if path{i}(end)~=1
                mark=0;
            end
        end
        if mark
            break
        end
    end
end