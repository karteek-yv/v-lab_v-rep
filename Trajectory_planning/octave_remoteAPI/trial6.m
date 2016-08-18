function trial6()
  clear 
  pkg load symbolic;
  pos_buf=zeros(3,10000);
  Link_len = [93,80,81,172];
  position(5)=0;
  Intl_P=[150,150,125];
  [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,Intl_P,Link_len), [0 0 0 0]); 
  %theta=position;
  val2=1;
  d = 0.5; %displacement step size
  dth=5; %theta step size in degrees
  j=1;
  while(val2)
    Intl_P=my_fun_fk(theta,Link_len);
    disp(Intl_P);
    P1=Intl_P;
    P2 = P1;
    P3 = [0,0,0];
    P4 = [0,0,0];
    
    Fnl_p=[0,0,0];
    %Fnl_P(1)=input('Enter target x-coorinate: ');
    %Fnl_P(2)=input('Enter target y-coorinate: ');
    %Fnl_P(3)=input('Enter target z-coorinate: ');
    Fnl_P=[150,-150,125];
    Obst_pos = [0,0,0];
    Obst_size = 0;
    Obst_pos = [200,0,60];
    Obst_size = 105;
    
    %Obst_pos(1)=input('Enter obstacle x-coorinate: ');
    %Obst_pos(2)=input('Enter obstacle y-coorinate: ');
    %Obst_pos(3)=input('Enter obstacle z-coorinate: ');
    %Obst_size = input('Enter obstacle size: ');
    syms x1 y1 z1
    P = [x1,y1,z1];
    %plane_1 = dot(normal, P-P1);
    %cir_1 = dot(P-(P1+P2)/2, P-(P1+P2)/2) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2);
    target = Intl_P;
    pos_buf(:,j)=target;j++;
    tic();
    while (dot(target-Intl_P, target-Intl_P)<dot(Intl_P-Fnl_P, Intl_P-Fnl_P))
      Unit_vec = (Fnl_P-target)/norm(Fnl_P-target); %unit vector in the direction
      target=target+Unit_vec*d;
      pos_buf(:,j)=target;j++;
      if (dot(target-Obst_pos, target-Obst_pos) < Obst_size^2)
        Z_vec = [0,0,1];
        cir_1 = dot(P-Obst_pos, P-Obst_pos) - dot(target-Obst_pos, target-Obst_pos);
        P1=target;
        P2=target+Unit_vec*d;
        while(dot(P2-Obst_pos, P2-Obst_pos) < Obst_size^2)
          P2=P2+Unit_vec*d;
        endwhile
        normal = cross(P1-P2, Z_vec);
        plane_1 = dot(normal, P-target);
        Unit_vec2=(P2-[P1(1),P1(2),P2(3)])/norm(P2-[P1(1),P1(2),P2(3)]);
        ang_1=180/pi*abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
        ang_2=180/pi*atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
        while(ang_1>dth && ang_2<90)
          fun_1 = dot(P-P1, P-P1) - dot(P1-Obst_pos, P1-Obst_pos)*2*(1-cos(dth*pi/180));
          %tic();
          s=solve(plane_1==0,cir_1==0,fun_1==0);
          %toc();
          %P1
          %P2
          P3=[double(s{1}.x1),double(s{1}.y1),double(s{1}.z1)];
          P4=[double(s{2}.x1),double(s{2}.y1),double(s{2}.z1)];
          ang_3=180/pi*abs(dot([Obst_pos(1),Obst_pos(2),P3(3)]-P3,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P3(3)]-P3)*atan2(norm(cross((P3-Obst_pos),Z_vec)),dot((P3-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
          ang_4=180/pi*abs(dot([Obst_pos(1),Obst_pos(2),P4(3)]-P4,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P4(3)]-P4)*atan2(norm(cross((P4-Obst_pos),Z_vec)),dot((P4-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
          if(ang_3<ang_4)
            P1=P3;          
          else
            P1=P4;
          endif
          pos_buf(:,j)=P1;j++;
          [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,P1,Link_len), [0 0 0 0]);     
          ang_1=180/pi*abs(dot([Obst_pos(1),Obst_pos(2),P1(3)]-P1,Unit_vec2)/norm([Obst_pos(1),Obst_pos(2),P1(3)]-P1)*atan2(norm(cross((P1-Obst_pos),Z_vec)),dot((P1-Obst_pos),Z_vec))+atan2(norm(cross((P2-Obst_pos),Z_vec)),dot((P2-Obst_pos),Z_vec)));
          ang_2=180/pi*atan2(norm(cross(Obst_pos-P1, Fnl_P-P1)),dot(Obst_pos-P1, Fnl_P-P1));
          if abs(fval) < 1e-3 && P1(3) > 0
            if theta(1) > pi/2
              theta(1) = -pi + theta(1);
              theta(2) = - theta(2);
              theta(3) = - theta(3);
              theta(4) = - theta(4);
            elseif theta(1) < -pi/2
              theta(1) = pi + theta(1);
              theta(2) = - theta(2);
              theta(3) = - theta(3);
              theta(4) = - theta(4);
            endif
            theta_deg = theta*180/pi;
          else
            %fprintf('Could not find soulution to given co-ordinates, please check \n')
            break;
          endif
          %pause(0.1);
        endwhile
        target=P1;
      endif
      pos_buf(:,j)=target;j++;
      %tic();
      [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,target,Link_len), [0 0 0 0]);
      %toc();
      if abs(fval) < 1e-3 && target(3) > 0
        if theta(1) > pi/2
          theta(1) = -pi + theta(1);
          theta(2) = - theta(2);
          theta(3) = - theta(3);
          theta(4) = - theta(4);
        elseif theta(1) < -pi/2
          theta(1) = pi + theta(1);
          theta(2) = - theta(2);
          theta(3) = - theta(3);
          theta(4) = - theta(4);
        endif
        theta_deg = theta*180/pi;
      else
        fprintf('Could not find soulution to given co-ordinates, please check \n')
      endif
      pause(0.1);
      %val2 = input('Enter 1 to reach new location and 0 to end program');
    endwhile     
    toc();
    val2 = input('Enter 1 to reach new location and 0 to end program');
  endwhile
  fid=fopen ('myfile.txt', 'w');
  for k=1:j-1
    fprintf(fid,'%f %f %f\n',pos_buf(1,k),pos_buf(2,k),pos_buf(3,k));
  end
  fclose(fid);
  figure();
  plot3(pos_buf(1,1:j-1),pos_buf(2,1:j-1),pos_buf(3,1:j-1),'r','LineWidth',1.5)
end
function fun1 = my_fun_ik (theta,target,Link_len)
  fun1 = zeros (3, 1);
	r=sqrt((target(1))^2 + (target(2))^2);
  fun1(1) = Link_len(1) + Link_len(2)*cos(theta(2)) + Link_len(3)*cos(theta(2)+theta(3)) + Link_len(4)*cos(theta(2)+theta(3)+theta(4)) - target(3);
	fun1(2) = Link_len(2)*sin(theta(2)) + Link_len(3)*sin(theta(2)+theta(3)) + Link_len(4)*sin(theta(2)+theta(3)+theta(4))- r;
	fun1(3) = theta(1) - atan2(target(2),target(1));
endfunction
function fun2 = my_fun_fk (theta,Link_len)
	r = Link_len(2)*sin(theta(2)) + Link_len(3)*sin(theta(2)+theta(3)) + Link_len(4)*sin(theta(2)+theta(3)+theta(4));
  fun2(1) = r*cos(theta(1));
  fun2(2) = r*sin(theta(1));
  fun2(3) = Link_len(1) + Link_len(2)*cos(theta(2)) + Link_len(3)*cos(theta(2)+theta(3)) + Link_len(4)*cos(theta(2)+theta(3)+theta(4));
  return;
endfunction
