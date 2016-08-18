clear
pkg load symbolic;
P1 = [125,125,125];
P2 = [325,-225,525];
P3 = [0,0,1];
normal = cross(P1-P2, P3);
syms x1 y1 z1
P = [x1,y1,z1];
plane_1 = dot(normal, P-P1)
cir_1 = dot(P-(P1+P2)/2, P-(P1+P2)/2) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2)
for i=1:5:30
%  i
%  fun_1 = dot(P-P1, P-P1) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2)*sin(i*pi/180)/sin(((180-i)/2)*(pi/180));
%  s=solve(plane_1==0,cir_1==0,fun_1==0);
%  if(s{1}.z1 > P1(3))
%    x=double(s{1}.x1)
%    y=double(s{1}.y1)
%    z=double(s{1}.z1)
%  else 
%    x=double(s{2}.x1)
%    y=double(s{2}.y1)
%    z=double(s{2}.z1)
%  endif
  tic();
  fun_1 = dot(P-P1, P-P1) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2)*2*(1-cos(i*pi/180))
  s=solve(plane_1==0,cir_1==0,fun_1==0);
   toc();
  if(double(s{1}.z1) > P1(3))
%    x=double(s{1}.x1)
%    y=double(s{1}.y1)
%    z=double(s{1}.z1)
    P4=[double(s{1}.x1),double(s{1}.y1),double(s{1}.z1)]
  else 
%    x=double(s{2}.x1)
%    y=double(s{2}.y1)
%    z=double(s{2}.z1)
    P4=[double(s{2}.x1),double(s{2}.y1),double(s{2}.z1)]
  endif
 
endfor

%for i=179.99:-1:170.00
%  i
%  fun_1 = dot(P-P2, P-P2) - dot(P2-(P1+P2)/2, P2-(P1+P2)/2)*sin(i*pi/180)/sin(((180-i)/2)*(pi/180));
%  s=solve(plane_1==0,cir_1==0,fun_1==0);
%  if(s{1}.z1 > P1(3))
%    x=double(s{1}.x1)
%    y=double(s{1}.y1)
%    z=double(s{1}.z1)
%  else 
%    x=double(s{2}.x1)
%    y=double(s{2}.y1)
%    z=double(s{2}.z1)
%  endif
%  fun_1 = dot(P-P2, P-P2) - dot(P2-(P1+P2)/2, P2-(P1+P2)/2)*2*(1-cos(i*pi/180));
%  s=solve(plane_1==0,cir_1==0,fun_1==0)
%  if(s{1}.z1 > P1(3))
%    x=double(s{1}.x1)
%    y=double(s{1}.y1)
%    z=double(s{1}.z1)
%  else 
%    x=double(s{2}.x1)
%    y=double(s{2}.y1)
%    z=double(s{2}.z1)
%  endif
%endfor