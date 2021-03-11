
function [itheta]=ikine_UR10(T)
    DH_UR10=load('DH_UR10.txt');
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
    nx=T(1,1);
    ny=T(2,1);
    nz=T(3,1);
    ox=T(1,2);
    oy=T(2,2);
    oz=T(3,2);
    ax=T(1,3);
    ay=T(2,3);
    az=T(3,3);
    px=T(1,4);
    py=T(2,4);
    pz=T(3,4);
    ithetat=zeros(1,6);
    itheta=cell(1,1);
    k=0;
    E=-(d2+d3+d4);
    F=px^2+py^2-E^2;
    if F<0
        error('out of working space')
    else
        theta1(1)=atan2(py,px)-atan2(E,F^0.5);
        theta1(2)=atan2(py,px)-atan2(E,-F^0.5);
        for m=1:2
            theta1t=theta1(m);
            if m==2
                if theta1(1)==theta1(2)
                    continue
                end
            end   
            sin5=((-sin(theta1t)*nx+cos(theta1t)*ny)^2+(-sin(theta1t)*ox+cos(theta1t)*oy)^2)^0.5;
            cos5=sin(theta1t)*ax-cos(theta1t)*ay;
            theta5(1)=atan2(sin5,cos5);
            theta5(2)=atan2(-sin5,cos5);
            %zheng
            for n=1:2
                theta5t=theta5(n);
                if n==2
                    sin5=-sin5;
                end
                if abs(sin5)<1e-8
                    error('theta5t=0 or pi,singular')
                else
                    sin6=(-sin(theta1t)*ox+cos(theta1t)*oy)/(-sin5);
                    cos6=(-sin(theta1t)*nx+cos(theta1t)*ny)/sin5;
                    theta6=atan2(sin6,cos6);
                end
                theta234=atan2(az/sin5,(cos(theta1t)*ax+ay*sin(theta1t))/sin5);
                M=pz-d1-d5*cos(theta234);
                N=px*cos(theta1t)+py*sin(theta1t)+d5*sin(theta234);
                A=2*a2*M;
                B=2*a2*N;
                C=a2^2-a3^2+M^2+N^2;
                if (A^2+B^2-C^2)<0
                    continue
                else
                    theta2(1)=atan2(B,-A)-atan2(C,-(A^2+B^2-C^2)^0.5);
                    theta2(2)=atan2(B,-A)-atan2(C,(A^2+B^2-C^2)^0.5);
                end
                for l=1:2
                    if l==2
                        if theta2(1)==theta2(2)
                            continue
                        end
                    end  
                    theta2t=theta2(l);
                    theta23=atan2((M-a2*sin(theta2t))/a3,(N-a2*cos(theta2t))/a3);
                    theta3=theta23-theta2t;
                    theta4=theta234-theta23;
                    ithetat(1)=theta1t/pi*180;
                    ithetat(2)=theta2t/pi*180;
                    ithetat(3)=theta3/pi*180;
                    ithetat(4)=theta4/pi*180;
                    ithetat(5)=theta5t/pi*180;
                    ithetat(6)=theta6/pi*180;
                    for i=1:6
                        if ithetat(i)>180
                            ithetat(i)=ithetat(i)-360;
                        elseif ithetat(i)<-180
                            ithetat(i)=ithetat(i)+360;
                        end
                    end
                    itheta{(m-1)*4+(n-1)*2+l}=ithetat;
                end
            end
        end
        
    end
end