function trial4()
clear
%pkg load symbolic;
%global l1 l2 l3 l4
%l1=93; l2=80; l3=81; l4=172; 
Link_len = [93,80,81,172];
  position(5)=0;
  %theta(5)=0;
      P2 = [0,0,0];
      P2(1)=input('Enter target x-coorinate: ');
      P2(2)=input('Enter target y-coorinate: ');
      P2(3)=input('Enter target z-coorinate: ');
      %plane_1 = dot(normal, P-P1);
      %cir_1 = dot(P-(P1+P2)/2, P-(P1+P2)/2) - dot(P1-(P1+P2)/2, P1-(P1+P2)/2);
      tic();
      target=P2;
        [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,target,Link_len), [0 0 0 0])
        if sum(abs(fval)) < 1e-3 && P2(3) > 0
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
          end
          theta_deg = theta*180/pi
        else
          fprintf('Could not find soulution to given co-ordinates, please check \n')
        end
        P1 = my_fun_fk(theta,Link_len)
        toc()
end
function fun1 = my_fun_ik (theta, target,Link_len)
  %global l1 l2 l3 l4 
  fun1 = zeros (3, 1);
	r=sqrt((target(1))^2 + (target(2))^2);
  fun1(1) = Link_len(1) + Link_len(2)*cos(theta(2)) + Link_len(3)*cos(theta(2)+theta(3)) + Link_len(4)*cos(theta(2)+theta(3)+theta(4)) - target(3);
	fun1(2) = Link_len(2)*sin(theta(2)) + Link_len(3)*sin(theta(2)+theta(3)) + Link_len(4)*sin(theta(2)+theta(3)+theta(4))- r;
	fun1(3) = theta(1) - atan2(target(2),target(1));
end
function fun2 = my_fun_fk (theta,Link_len)
  %global l1 l2 l3 l4 
	r = Link_len(2)*sin(theta(2)) + Link_len(3)*sin(theta(2)+theta(3)) + Link_len(4)*sin(theta(2)+theta(3)+theta(4));
  fun2(1) = r*cos(theta(1));
  fun2(2) = r*sin(theta(1));
  fun2(3) = Link_len(1) + Link_len(2)*cos(theta(2)) + Link_len(3)*cos(theta(2)+theta(3)) + Link_len(4)*cos(theta(2)+theta(3)+theta(4));
  return;
end