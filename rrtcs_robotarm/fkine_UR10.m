%% 函数说明
%******************************
%函数名称：fkine_UR5
%函数功能：UR10机器人正解
%函数输入：各个关节角度，单位是角度，为1×6向量
%函数输出：末端姿态，为4×4矩阵
%修改时间：201800403
%******************************

%% 根据建立的DH参数得到机器人正解
function [T]=fkine_UR10(theta,IfAlphabet)
 %IfAlpahabet为真(非0)表示结果用字母显示出来，此时无论输入任何角度将被屏蔽；为假(0)表示用实数显示出来
 DH_UR10=load('DH_UR10.txt');
if IfAlphabet
        syms theta1 theta2 theta3 theta4 theta5 theta6;
        syms a2 a3;
        syms d1 d2 d3 d4 d5;
        afa0=DH_UR10(1,1);
        afa1=DH_UR10(2,1);
        afa2=DH_UR10(3,1);
        afa3=DH_UR10(4,1);
        afa4=DH_UR10(5,1);
        afa5=DH_UR10(6,1);
        a0=DH_UR10(1,2);
        a1=DH_UR10(2,2);
        a4=DH_UR10(5,2);
        a5=DH_UR10(6,2);
        d6=DH_UR10(6,3);
else
        theta1=theta(1)*pi/180;
        theta2=theta(2)*pi/180;
        theta3=theta(3)*pi/180;
        theta4=theta(4)*pi/180;
        theta5=theta(5)*pi/180;
        theta6=theta(6)*pi/180;
        afa0=DH_UR10(1,1);
        afa1=DH_UR10(2,1);
        afa2=DH_UR10(3,1);
        afa3=DH_UR10(4,1);
        afa4=DH_UR10(5,1);
        afa5=DH_UR10(6,1);
        a0=DH_UR10(1,2);
        a1=DH_UR10(2,2);
        a2=DH_UR10(3,2);
        a3=DH_UR10(4,2);
        a4=DH_UR10(5,2);
        a5=DH_UR10(6,2);
        d1=DH_UR10(1,3);
        d2=DH_UR10(2,3);
        d3=DH_UR10(3,3);
        d4=DH_UR10(4,3);
        d5=DH_UR10(5,3);
        d6=DH_UR10(6,3);
end
T01=[cos(theta1),-sin(theta1),0,a0;sin(theta1)*cos(afa0/180*pi),cos(theta1)*cos(afa0/180*pi),-sin(afa0/180*pi),-sin(afa0/180*pi)*d1;sin(theta1)*sin(afa0/180*pi),cos(theta1)*sin(afa0/180*pi),cos(afa0/180*pi),cos(afa0/180*pi)*d1;0,0,0,1];
T12=[cos(theta2),-sin(theta2),0,a1;sin(theta2)*cos(afa1/180*pi),cos(theta2)*cos(afa1/180*pi),-sin(afa1/180*pi),-sin(afa1/180*pi)*d2;sin(theta2)*sin(afa1/180*pi),cos(theta2)*sin(afa1/180*pi),cos(afa1/180*pi),cos(afa1/180*pi)*d2;0,0,0,1];
T23=[cos(theta3),-sin(theta3),0,a2;sin(theta3)*cos(afa2/180*pi),cos(theta3)*cos(afa2/180*pi),-sin(afa2/180*pi),-sin(afa2/180*pi)*d3;sin(theta3)*sin(afa2/180*pi),cos(theta3)*sin(afa2/180*pi),cos(afa2/180*pi),cos(afa2/180*pi)*d3;0,0,0,1];
T34=[cos(theta4),-sin(theta4),0,a3;sin(theta4)*cos(afa3/180*pi),cos(theta4)*cos(afa3/180*pi),-sin(afa3/180*pi),-sin(afa3/180*pi)*d4;sin(theta4)*sin(afa3/180*pi),cos(theta4)*sin(afa3/180*pi),cos(afa3/180*pi),cos(afa3/180*pi)*d4;0,0,0,1];
T45=[cos(theta5),-sin(theta5),0,a4;sin(theta5)*cos(afa4/180*pi),cos(theta5)*cos(afa4/180*pi),-sin(afa4/180*pi),-sin(afa4/180*pi)*d5;sin(theta5)*sin(afa4/180*pi),cos(theta5)*sin(afa4/180*pi),cos(afa4/180*pi),cos(afa4/180*pi)*d5;0,0,0,1];
T56=[cos(theta6),-sin(theta6),0,a5;sin(theta6)*cos(afa5/180*pi),cos(theta6)*cos(afa5/180*pi),-sin(afa5/180*pi),-sin(afa5/180*pi)*d6;sin(theta6)*sin(afa5/180*pi),cos(theta6)*sin(afa5/180*pi),cos(afa5/180*pi),cos(afa5/180*pi)*d6;0,0,0,1];
T06=T01*T12*T23*T34*T45*T56;
T16=T12*T23*T34*T45*T56;
T=cell(1,7);
T{1}=T06;
T{2}=T01;
T{3}=T12;
T{4}=T23;
T{5}=T34;
T{6}=T45;
T{7}=T56;
T{8}=T16;
end