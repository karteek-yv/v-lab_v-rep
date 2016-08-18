function trial9()
  Link_len = [93,80,81,172];
  Fnl_P(1)=input('Enter target x-coorinate: ');
  Fnl_P(2)=input('Enter target y-coorinate: ');
  Fnl_P(3)=input('Enter target z-coorinate: ');
  target=Fnl_P;
  [theta, fval, info] = fsolve (@(theta)my_fun_ik(theta,target,Link_len), [0 0 0 0]);
  theta_deg = theta*180/pi
endfunction

function fun1 = my_fun_ik (theta,target,Link_len)
  fun1 = zeros (3, 1);
	r=sqrt((target(1))^2 + (target(2))^2);
  fun1(1) = Link_len(1) + Link_len(2)*cos(theta(2)) + Link_len(3)*cos(theta(2)+theta(3)) + Link_len(4)*cos(theta(2)+theta(3)+theta(4)) - target(3);
	fun1(2) = Link_len(2)*sin(theta(2)) + Link_len(3)*sin(theta(2)+theta(3)) + Link_len(4)*sin(theta(2)+theta(3)+theta(4))- r;
	fun1(3) = theta(1) - atan2(target(2),target(1));
endfunction