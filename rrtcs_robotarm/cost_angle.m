function [cost]=cost_angle(PathV)
cost=0;
for i=2:size(PathV,1)
    cost=cost+norm(PathV(i-1,:)-PathV(i,:));
end