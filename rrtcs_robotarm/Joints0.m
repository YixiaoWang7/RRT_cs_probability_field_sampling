function [SetPointS]=Joints0(BasePoint,angle,T)
SetPoint=cell(1,7);%DH origins
SetPointT=cell(1,7);% DH origins T
SetPoint{1}=[0,0,0]';
SetPointT{1}=[BasePoint;1];
for i=1:6
    Ttemp=T{2};
    for j=2:i
        Ttemp=Ttemp*T{j+1};
    end
    SetPointT{i+1}=Ttemp(:,4);
    SetPoint{i+1}=Ttemp(1:3,4);
end
SetPointS=cell(1,9);%joints of the arm
for i=1:3
    SetPointS{i}=SetPoint{i};
end
PT=T{2}*T{3}*[612;0;0;1];
SetPointS{4}=PT(1:3,:);
SetPointS{5}=SetPoint{4};
PT=T{2}*T{3}*T{4}*[572;0;0;1];
SetPointS{6}=PT(1:3,:);
for i=5:7
    SetPointS{i+2}=SetPoint{i};
end
PT=T{2}*T{3}*T{4}*T{5}*T{6}*T{7}*[0;0;92;1];
SetPointS{10}=PT(1:3,:);

for i=1:10
    SetPointS{i}(1:2,1)=[cos(angle) -sin(angle);sin(angle) cos(angle) ]*SetPointS{i}(1:2,1);
    SetPointS{i}=SetPointS{i}+BasePoint;
end