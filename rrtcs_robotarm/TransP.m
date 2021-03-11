function [p2,R]=TransP(p1,Rxyz,p0,flag)
%flag=0 angle
%flag=1 vector
if flag==0
    angle=Rxyz;
    Rx=[1 0 0;0 cos(angle(1)) -sin(angle(1));0 sin(angle(1)) cos(angle(1))];
    Ry=[cos(angle(2)) 0 sin(angle(2));0 1 0;-sin(angle(2)) 0 cos(angle(2))];
    Rz=[cos(angle(3)) -sin(angle(3)) 0 ;sin(angle(3)) cos(angle(3)) 0; 0 0 1];
    R=Rz*Ry*Rx;
    p2=R*p1+p0;
elseif flag==1
    V=Rxyz;
    theta=norm(V);
    n=V/theta;
    nx=n(1);
    ny=n(2);
    nz=n(3);
    R=[nx^2*(1-cos(theta))+cos(theta) nx*ny*(1-cos(theta))-nz*sin(theta) nx*nz*(1-cos(theta))+ny*sin(theta);nx*ny*(1-cos(theta))+nz*sin(theta) ny^2*(1-cos(theta))+cos(theta) nz*ny*(1-cos(theta))-nx*sin(theta);nx*nz*(1-cos(theta))-ny*sin(theta) nz*ny*(1-cos(theta))+nx*sin(theta) nz*nz*(1-cos(theta))+cos(theta)];
    p2=R*p1+p0;
else
    error('flag must be 0 or 1.')
end