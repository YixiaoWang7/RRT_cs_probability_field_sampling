function [R]=TransR(angle)
Rx=[1 0 0;0 cos(angle(1)) -sin(angle(1));0 sin(angle(1)) cos(angle(1))];
Ry=[cos(angle(2)) 0 sin(angle(2));0 1 0;-sin(angle(2)) 0 cos(angle(2))];
Rz=[cos(angle(3)) -sin(angle(3)) 0 ;sin(angle(3)) cos(angle(3)) 0; 0 0 1];
R=Rz*Ry*Rx;