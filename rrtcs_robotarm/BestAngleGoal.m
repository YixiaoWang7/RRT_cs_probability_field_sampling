function [AngleGoalF]=BestAngleGoal(Rangle,ActP0,ActP6,AngleStart,ob,BasePoint,angle)
R=TransR(Rangle);
PT06=ActP0-R*ActP6;
T06=[R PT06;0 0 0 1];
[itheta]=ikine_UR10(T06);
AngleGoal=cell(1,1);
l=1;
for i=1:8
    if ~isempty(itheta{i})
        if ~isCrash(ob,itheta{i},BasePoint,angle)
            AngleGoal{l}=itheta{i};
            l=l+1;
%             T{i}
%             T=fkine_UR10(itheta{i},0);
%             [SetPointS]=Joints0(BasePoint,angle,T);
%             draw_arm(SetPointS)
        end
    end
end
DisAngle=zeros(1,length(AngleGoal));
for i=1:length(AngleGoal)
    DisAngle(i)=norm(AngleGoal{i}-AngleStart);
end
DisMinIndex=find(DisAngle==min(DisAngle));%it can be showed
AngleGoalF=AngleGoal{DisMinIndex};