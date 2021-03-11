function [h]=draw_armA(SetPointS,a)
for i=1:9
    p1=SetPointS{i};
    p2=SetPointS{i+1};
    h(i)=plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],a);
    hold on
end













