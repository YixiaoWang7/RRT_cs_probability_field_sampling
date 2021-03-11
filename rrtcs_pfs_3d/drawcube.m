function drawcube(center,sidelength,color,facealpha)
difference=sidelength*[-0.5 -0.5 0.5 0.5 -0.5 -0.5 0.5 0.5;-0.5 0.5 0.5 -0.5 -0.5 0.5 0.5 -0.5;-0.5 -0.5 -0.5 -0.5 0.5 0.5 0.5 0.5];
BoxPoint=[];
for j=1:8
    BoxPoint=[BoxPoint center+difference(:,j)];
end
for j=1:2
    h=patch(BoxPoint(1,(j-1)*4+1:j*4),BoxPoint(2,(j-1)*4+1:j*4),BoxPoint(3,(j-1)*4+1:j*4),color);
    set(h,'edgecolor','k','facealpha',facealpha)
end
for j=1:3
    h=patch(BoxPoint(1,[1 2 6 5]+j-1),BoxPoint(2,[1 2 6 5]+j-1),BoxPoint(3,[1 2 6 5]+j-1),color);
    set(h,'edgecolor','k','facealpha',facealpha)
end
h=patch(BoxPoint(1,[4 1 5 8]),BoxPoint(2,[4 1 5 8]),BoxPoint(3,[4 1 5 8]),color);
set(h,'edgecolor','k','facealpha',facealpha)
end