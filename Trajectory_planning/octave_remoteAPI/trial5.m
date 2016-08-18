function trial5()
pkg load symbolic;
P1=[125,50,125];
P2=[125,-50,125];
syms x1 y1 z1
P = [x1,y1,z1];
z=[0,0,1];
normal = cross(P1-P2, z)
plane_1 = dot(normal, P-P1)
cir_1 = dot(P-(P1+P2)/2, P-(P1+P2)/2) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2)
angle1=ang_fun(P1,P2)
P2=[-sqrt(2),sqrt(2),2];
P1=[sqrt(2),-sqrt(2),2];
angle1=ang_fun(P1,P2)
P2=[-sqrt(2),sqrt(2),-2];
P1=[sqrt(2),-sqrt(2),2];
angle2=ang_fun(P1,P2)
P2=[sqrt(2),-sqrt(2),2];
P1=[-2,2,0];
angle3=ang_fun(P1,P2)
P2=[sqrt(2),-sqrt(2),-2];
P1=[-2,2,0];
angle4=ang_fun(P1,P2)
endfunction
function ang_1=ang_fun(P1,P2)
z=[0,0,1];
c=[125,0,125];
P1=P1-c;
P2=P2-c;
%z=[(P1(1)+P2(1))/2,(P1(2)+P2(2))/2,1];
ang_1(1)=180/pi*(atan2(norm(cross(P1,z)),dot(P1,z)));
ang_1(2)=180/pi*(atan2(norm(cross(P2,z)),dot(P2,z)));
%ang_1(3)=180/pi*(atan2(norm(cross(P1,z)),dot(P1,z))+sign(dot(z-P1,P2-z))*atan2(norm(cross(P2,z)),dot(P2,z)));
ang_1(3)=180/pi*abs(sign(dot(z-P1,P2-z))*atan2(norm(cross(P1,z)),dot(P1,z))+atan2(norm(cross(P2,z)),dot(P2,z)));
endfunction